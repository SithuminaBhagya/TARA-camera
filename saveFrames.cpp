#include "GalaxyIncludes.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <atomic>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <chrono>
#include <conio.h>
#include <malloc.h>
#include <cstdint>

namespace fs = std::filesystem;

// index 0 = master (Camera 1), indices 1-3 = slaves (Cameras 2-4)
const std::vector<std::string> CAM_SNS = {
    "JCK26010021",  // Camera 1 — master
    "JCK26010020",  // Camera 2 — slave
    "JCK26010019",  // Camera 3 — slave
    "JCK26010013"   // Camera 4 — slave
};

const std::string SAVE_ROOT  = "recordings";
const int         IMG_W      = 2600;
const int         IMG_H      = 2160;
const size_t      FRAME_BYTES = (size_t)IMG_W * IMG_H;

// Test configuration: 30 fps Mono8 raw for 20 seconds.
// Combined data rate = 4 * 30 * 5.36 MB = ~643 MB/s.
// Total data = ~12.9 GB, fits comfortably in the SLC cache window.
const double CAPTURE_FPS   = 30.0;
const int    RECORD_SECONDS = 20;

// ── Item passed from callback to writer thread ─────────────────────
struct WriteItem
{
    int      camIdx;
    int      bufIdx;
    int      frameIndex;
    uint64_t timestamp;
};

// ── Per-camera state ──────────────────────────────────────────────
struct CameraState
{
    std::atomic<int>  frameCount{ 0 };
    std::atomic<int>  savedCount{ 0 };
    std::atomic<int>  droppedCount{ 0 };
    std::string       savePath;

    HANDLE        hFrames{ INVALID_HANDLE_VALUE };
    std::ofstream tsFile;

    // Raw frame buffer pool — FRAME_BYTES each.
    // Callback grabs a free bufIdx, memcpy's raw pixels in.
    // Writer reads from bufs[bufIdx], then returns bufIdx to freeBufs.
    std::vector<uint8_t*> bufs;
    std::vector<int>      freeBufs;
    std::mutex            freeBufsMtx;

    // 128 buffers × 5.36 MB = ~686 MB per camera (~2.7 GB total).
    // Absorption buffer if writer momentarily lags behind capture.
    static const int MAX_QUEUE_DEPTH = 128;

    CameraState() = default;
    CameraState(const CameraState&) = delete;
    CameraState& operator=(const CameraState&) = delete;
};

CameraState g_cameras[4];

// ── Global write queue (raw frames from all cameras, interleaved) ──
std::deque<WriteItem>      g_writeQueue;
std::mutex                 g_queueMtx;
std::condition_variable    g_queueCv;
std::atomic<bool>          g_stopWriter{ false };
std::thread                g_writerThread;

// ── Single writer thread — fixed-size raw writes ──────────────────
void writerThreadFunc()
{
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    double totalWriteMs[4] = {};
    double maxWriteMs[4]   = {};
    size_t totalBytes[4]   = {};
    int    writeCount[4]   = {};

    while (true)
    {
        WriteItem item;
        {
            std::unique_lock<std::mutex> lk(g_queueMtx);
            g_queueCv.wait(lk, [] {
                return !g_writeQueue.empty() || g_stopWriter.load();
            });
            if (g_writeQueue.empty())
                break;
            item = g_writeQueue.front();
            g_writeQueue.pop_front();
        }

        int   ci  = item.camIdx;
        auto& cam = g_cameras[ci];

        DWORD written = 0;
        auto t0 = std::chrono::steady_clock::now();
        WriteFile(cam.hFrames, cam.bufs[item.bufIdx],
                  (DWORD)FRAME_BYTES, &written, nullptr);
        double ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0).count();

        totalWriteMs[ci] += ms;
        if (ms > maxWriteMs[ci]) maxWriteMs[ci] = ms;
        totalBytes[ci] += FRAME_BYTES;
        writeCount[ci]++;

        int savedIdx = cam.savedCount.fetch_add(1);
        cam.tsFile << savedIdx << ","
                   << item.frameIndex << ","
                   << item.timestamp  << "\n";

        // Release buffer back to camera's free pool
        {
            std::lock_guard<std::mutex> lk(cam.freeBufsMtx);
            cam.freeBufs.push_back(item.bufIdx);
        }
    }

    for (int i = 0; i < 4; ++i)
    {
        FlushFileBuffers(g_cameras[i].hFrames);
        g_cameras[i].tsFile.flush();
    }

    std::cout << "\nWrite statistics:" << std::endl;
    for (int i = 0; i < 4; ++i)
    {
        if (writeCount[i] > 0)
        {
            double avgMs   = totalWriteMs[i] / writeCount[i];
            double totalMB = (double)totalBytes[i] / 1024.0 / 1024.0;
            std::cout << "  Cam" << (i + 1) << " write: avg " << std::fixed
                      << std::setprecision(2) << avgMs << " ms  max "
                      << maxWriteMs[i] << " ms  total "
                      << std::setprecision(1) << totalMB << " MB" << std::endl;
        }
    }
}

// ── Callback — must return fast ──────────────────────────────────
class CCaptureHandler : public ICaptureEventHandler
{
public:
    int m_index;
    CCaptureHandler(int index) : m_index(index) {}

    void DoOnImageCaptured(CImageDataPointer& imgPtr, void* /*pUserParam*/) override
    {
        if (imgPtr->GetStatus() != GX_FRAME_STATUS_SUCCESS)
            return;

        uint64_t ts    = imgPtr->GetTimeStamp();
        void*    pRaw8 = imgPtr->ConvertToRaw8(GX_BIT_0_7);

        auto& cam   = g_cameras[m_index];
        int   count = cam.frameCount.fetch_add(1);

        int bufIdx = -1;
        {
            std::lock_guard<std::mutex> lk(cam.freeBufsMtx);
            if (!cam.freeBufs.empty())
            {
                bufIdx = cam.freeBufs.back();
                cam.freeBufs.pop_back();
            }
        }

        if (bufIdx < 0)
        {
            cam.droppedCount.fetch_add(1);
            return;
        }

        memcpy(cam.bufs[bufIdx], pRaw8, FRAME_BYTES);

        // Push directly to global write queue (no compression stage)
        {
            std::lock_guard<std::mutex> lk(g_queueMtx);
            g_writeQueue.push_back({ m_index, bufIdx, count, ts });
        }
        g_queueCv.notify_one();
    }
};

// ── Folder + file creation ────────────────────────────────────────
std::string createExperimentFolder()
{
    fs::create_directories(SAVE_ROOT);

    int expNum = 1;
    std::string expPath;
    do {
        std::ostringstream ss;
        ss << SAVE_ROOT << "/experiment_"
           << std::setw(3) << std::setfill('0') << expNum++;
        expPath = ss.str();
    } while (fs::exists(expPath));

    for (int i = 0; i < 4; ++i)
    {
        std::string path = expPath + "/camera_" + std::to_string(i + 1);
        fs::create_directories(path);
        g_cameras[i].savePath = path;

        // Single growing frames.bin per camera — buffered writes.
        // No chunk rotation: we expect 12.9 GB total to fit in the SLC cache
        // window so the OS page cache and disk shouldn't bottleneck.
        g_cameras[i].hFrames = CreateFileA(
            (path + "/frames.bin").c_str(),
            GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN,
            nullptr);

        g_cameras[i].tsFile.open(path + "/timestamps.csv");
        g_cameras[i].tsFile << "saved_index,frame_index,timestamp_ticks\n";

        // Raw frame buffer pool
        g_cameras[i].bufs.reserve(CameraState::MAX_QUEUE_DEPTH);
        for (int j = 0; j < CameraState::MAX_QUEUE_DEPTH; ++j)
        {
            auto* buf = static_cast<uint8_t*>(malloc(FRAME_BYTES));
            g_cameras[i].bufs.push_back(buf);
            g_cameras[i].freeBufs.push_back(j);
        }

        // Metadata: format=raw with fixed FRAME_BYTES stride.
        // Compatible with playback.cpp's existing raw mode.
        std::ofstream meta(path + "/metadata.txt");
        meta << "width="        << IMG_W       << "\n"
             << "height="       << IMG_H       << "\n"
             << "channels=1\n"
             << "dtype=uint8\n"
             << "format=raw\n"
             << "frame_stride=" << FRAME_BYTES << "\n"
             << "fps="          << CAPTURE_FPS << "\n";
    }

    return expPath;
}

// ── Trigger configuration ─────────────────────────────────────────
void configureMaster(CGXFeatureControlPointer& fc)
{
    fc->GetEnumFeature("TriggerMode")->SetValue("Off");
    fc->GetEnumFeature("LineSelector")->SetValue("Line1");
    fc->GetEnumFeature("LineMode")->SetValue("Output");
    fc->GetEnumFeature("LineSource")->SetValue("ExposureActive");
}

void configureSlave(CGXFeatureControlPointer& fc)
{
    fc->GetEnumFeature("TriggerSelector")->SetValue("FrameStart");
    fc->GetEnumFeature("TriggerMode")->SetValue("On");
    fc->GetEnumFeature("TriggerSource")->SetValue("Line0");
    fc->GetEnumFeature("TriggerActivation")->SetValue("RisingEdge");
    fc->GetEnumFeature("AcquisitionFrameRateMode")->SetValue("Off");
}

// ── Main ──────────────────────────────────────────────────────────
int main()
{
    try
    {
        IGXFactory::GetInstance().Init();
        std::cout << "SDK Initialized." << std::endl;

        GxIAPICPP::gxdeviceinfo_vector vecDeviceInfo;
        IGXFactory::GetInstance().UpdateDeviceList(1000, vecDeviceInfo);
        std::cout << "Found " << vecDeviceInfo.size() << " camera(s)." << std::endl;

        std::string expPath = createExperimentFolder();
        std::cout << "Saving to: " << expPath << std::endl;
        std::cout << "Mode: " << IMG_W << "x" << IMG_H << " 8-bit mono RAW @ "
                  << CAPTURE_FPS << " fps for " << RECORD_SECONDS << " sec" << std::endl;
        std::cout << "Expected: " << (4 * CAPTURE_FPS * FRAME_BYTES / 1024.0 / 1024.0)
                  << " MB/s combined, "
                  << (4 * CAPTURE_FPS * FRAME_BYTES * RECORD_SECONDS / 1024.0 / 1024.0 / 1024.0)
                  << " GB total\n" << std::endl;

        g_writerThread = std::thread(writerThreadFunc);

        std::vector<CGXDevicePointer>                  cameras(4);
        std::vector<CGXStreamPointer>                  streams(4);
        std::vector<CGXFeatureControlPointer>          featureControls(4);
        std::vector<std::shared_ptr<CCaptureHandler>>  callbacks(4);

        for (int i = 0; i < 4; ++i)
        {
            cameras[i] = IGXFactory::GetInstance().OpenDeviceBySN(
                GxIAPICPP::gxstring(CAM_SNS[i].c_str()), GX_ACCESS_EXCLUSIVE
            );

            featureControls[i] = cameras[i]->GetRemoteFeatureControl();

            featureControls[i]->GetIntFeature("Width")->SetValue(IMG_W);
            featureControls[i]->GetIntFeature("Height")->SetValue(IMG_H);
            featureControls[i]->GetIntFeature("OffsetX")->SetValue(0);
            featureControls[i]->GetIntFeature("OffsetY")->SetValue(0);
            featureControls[i]->GetIntFeature("DeviceLinkThroughputLimit")->SetValue(400000000);
            featureControls[i]->GetEnumFeature("AcquisitionMode")->SetValue("Continuous");

            if (i == 0)
            {
                featureControls[i]->GetEnumFeature("AcquisitionFrameRateMode")->SetValue("On");
                featureControls[i]->GetFloatFeature("AcquisitionFrameRate")->SetValue(CAPTURE_FPS);
                configureMaster(featureControls[i]);
            }
            else
                configureSlave(featureControls[i]);

            streams[i] = cameras[i]->OpenStream(0);
            callbacks[i] = std::make_shared<CCaptureHandler>(i);
            streams[i]->RegisterCaptureCallback(callbacks[i].get(), nullptr);

            std::string role = (i == 0) ? "master" : "slave";
            std::cout << "Camera " << (i + 1) << " (" << CAM_SNS[i]
                      << ", " << role << ") configured." << std::endl;
        }

        for (int i = 0; i < 4; ++i)
            streams[i]->StartGrab();

        for (int i = 1; i < 4; ++i)
        {
            featureControls[i]->GetCommandFeature("AcquisitionStart")->Execute();
            std::cout << "Camera " << (i + 1) << " armed." << std::endl;
        }

        featureControls[0]->GetCommandFeature("AcquisitionStart")->Execute();
        auto recordStart = std::chrono::steady_clock::now();
        std::cout << "Camera 1 started.\n" << std::endl;
        std::cout << "Recording... (auto-stops at " << RECORD_SECONDS
                  << " sec, or press Q to stop early)\n" << std::endl;

        while (true)
        {
            if (_kbhit())
            {
                char c = _getch();
                if (c == 'q' || c == 'Q')
                    break;
            }

            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - recordStart).count();
            if (elapsed >= RECORD_SECONDS * 1000)
                break;

            size_t writeQ;
            {
                std::lock_guard<std::mutex> lock(g_queueMtx);
                writeQ = g_writeQueue.size();
            }
            std::cout << "\r  t=" << std::fixed << std::setprecision(1)
                      << (elapsed / 1000.0) << "s  Write queue: " << writeQ
                      << "  saved: ["
                      << g_cameras[0].savedCount.load() << "/"
                      << g_cameras[1].savedCount.load() << "/"
                      << g_cameras[2].savedCount.load() << "/"
                      << g_cameras[3].savedCount.load() << "]      "
                      << std::flush;

            Sleep(200);
        }

        std::cout << "\n\nStopping acquisition..." << std::endl;
        featureControls[0]->GetCommandFeature("AcquisitionStop")->Execute();
        for (int i = 1; i < 4; ++i)
            featureControls[i]->GetCommandFeature("AcquisitionStop")->Execute();

        for (int i = 0; i < 4; ++i)
        {
            streams[i]->StopGrab();
            streams[i]->UnregisterCaptureCallback();
            streams[i]->Close();
            cameras[i]->Close();
        }

        std::cout << "Flushing write queue (waiting for disk)..." << std::endl;
        {
            std::lock_guard<std::mutex> lock(g_queueMtx);
            g_stopWriter.store(true);
            g_queueCv.notify_one();
        }
        g_writerThread.join();

        for (int i = 0; i < 4; ++i)
        {
            CloseHandle(g_cameras[i].hFrames);
            g_cameras[i].tsFile.close();

            std::ofstream meta(g_cameras[i].savePath + "/metadata.txt", std::ios::app);
            meta << "frames=" << g_cameras[i].savedCount.load() << "\n";

            for (auto* buf : g_cameras[i].bufs)
                free(buf);
            g_cameras[i].bufs.clear();
        }

        IGXFactory::GetInstance().Uninit();

        std::cout << "\nResults:" << std::endl;
        bool allMatch = true;
        int firstCount = g_cameras[0].savedCount.load();
        for (int i = 0; i < 4; ++i)
        {
            std::string role = (i == 0) ? "master" : "slave ";
            int saved   = g_cameras[i].savedCount.load();
            int dropped = g_cameras[i].droppedCount.load();
            if (saved != firstCount) allMatch = false;
            std::cout << "  Camera " << (i + 1) << " (" << role << " "
                      << CAM_SNS[i] << "): " << saved << " saved";
            if (dropped > 0)
                std::cout << ",  " << dropped << " dropped (queue full)";
            std::cout << std::endl;
        }
        std::cout << (allMatch ? "  Frame count match: YES" : "  Frame count match: NO") << std::endl;
        std::cout << "Saved to: " << expPath << std::endl;
    }
    catch (CGalaxyException& e)
    {
        std::cout << "Camera error: " << e.what() << std::endl;
        return -1;
    }
    catch (std::exception& e)
    {
        std::cout << "Error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
