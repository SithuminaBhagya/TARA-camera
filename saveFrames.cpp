#include "GalaxyIncludes.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <atomic>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <conio.h>
#include <malloc.h>

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

// FILE_FLAG_NO_BUFFERING requires writes to be a multiple of the sector size (4096).
// Pad FRAME_BYTES up to the next 4096-byte boundary.
// Playback seeks by FRAME_STRIDE but reads only FRAME_BYTES valid pixels.
const size_t SECTOR        = 4096;
const size_t FRAME_STRIDE  = (FRAME_BYTES + SECTOR - 1) / SECTOR * SECTOR; // 5,619,712

// ── Frame item passed from callback to writer thread ──────────────
struct FrameItem
{
    int      bufIdx;      // index into CameraState::bufs / matPool
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

    // FILE_FLAG_NO_BUFFERING: bypasses OS write-back cache, eliminating
    // dirty-page throttle stalls (which caused 30-second WriteFile pauses).
    // Requires 4096-aligned buffers and write sizes that are multiples of 4096.
    HANDLE hFrames{ INVALID_HANDLE_VALUE };

    std::ofstream tsFile;

    // Pre-allocated, sector-aligned frame buffer pool.
    // bufs[i]    — raw 4096-aligned pointer used for WriteFile and memcpy
    // matPool[i] — cv::Mat header pointing into bufs[i] (does not own memory)
    // freeBufs   — indices of buffers currently not in writeQueue
    std::vector<uint8_t*> bufs;
    std::vector<cv::Mat>  matPool;
    std::vector<int>      freeBufs;

    std::queue<FrameItem>    writeQueue;
    std::mutex               queueMtx;
    std::condition_variable  queueCv;
    std::atomic<bool>        stopWriter{ false };
    std::thread              writerThread;

    // 48 frames × 5.6 MB = ~268 MB per camera.
    static const int MAX_QUEUE_DEPTH = 48;

    CameraState() = default;
    CameraState(const CameraState&) = delete;
    CameraState& operator=(const CameraState&) = delete;
};

CameraState g_cameras[4];

// ── Writer thread — one per camera ───────────────────────────────
void writerThreadFunc(int camIdx)
{
    auto& cam = g_cameras[camIdx];
    int    lastBufIdx   = -1;
    double totalWriteMs = 0.0;
    double maxWriteMs   = 0.0;

    while (true)
    {
        FrameItem item;
        {
            std::unique_lock<std::mutex> lock(cam.queueMtx);
            if (lastBufIdx >= 0)
            {
                cam.freeBufs.push_back(lastBufIdx);
                lastBufIdx = -1;
            }
            cam.queueCv.wait(lock, [&]{
                return !cam.writeQueue.empty() || cam.stopWriter.load();
            });

            if (cam.writeQueue.empty())
                break;

            item = cam.writeQueue.front();
            cam.writeQueue.pop();
        }

        DWORD written;
        auto  t0 = std::chrono::steady_clock::now();
        WriteFile(cam.hFrames, cam.bufs[item.bufIdx],
                  (DWORD)FRAME_STRIDE, &written, nullptr);
        double ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0).count();
        totalWriteMs += ms;
        if (ms > maxWriteMs) maxWriteMs = ms;

        int savedIdx = cam.savedCount.fetch_add(1);
        cam.tsFile << savedIdx << "," << item.frameIndex << "," << item.timestamp << "\n";

        lastBufIdx = item.bufIdx;
    }

    if (lastBufIdx >= 0)
    {
        std::lock_guard<std::mutex> lock(cam.queueMtx);
        cam.freeBufs.push_back(lastBufIdx);
    }

    FlushFileBuffers(cam.hFrames);
    cam.tsFile.flush();

    int saved = cam.savedCount.load();
    if (saved > 0)
    {
        double avgMs = totalWriteMs / saved;
        double mbps  = (FRAME_STRIDE / 1024.0 / 1024.0) / (avgMs / 1000.0);
        std::cout << "  Cam" << (camIdx + 1) << " write: avg " << std::fixed
                  << std::setprecision(2) << avgMs << " ms  max " << maxWriteMs
                  << " ms  (" << (int)mbps << " MB/s)" << std::endl;
    }
}

// ── Callback — must return fast ──────────────────────────────────
class CCaptureHandler : public ICaptureEventHandler
{
public:
    int m_index;
    CCaptureHandler(int index) : m_index(index) {}

    void DoOnImageCaptured(CImageDataPointer& imgPtr, void* pUserParam) override
    {
        if (imgPtr->GetStatus() != GX_FRAME_STATUS_SUCCESS)
            return;

        uint64_t ts    = imgPtr->GetTimeStamp();
        void*    pRaw8 = imgPtr->ConvertToRaw8(GX_BIT_0_7);

        auto& cam   = g_cameras[m_index];
        int   count = cam.frameCount.fetch_add(1);

        {
            std::lock_guard<std::mutex> lock(cam.queueMtx);
            if (!cam.freeBufs.empty())
            {
                int idx = cam.freeBufs.back();
                cam.freeBufs.pop_back();
                memcpy(cam.bufs[idx], pRaw8, FRAME_BYTES);
                cam.writeQueue.push({ idx, count, ts });
                cam.queueCv.notify_one();
            }
            else
            {
                cam.droppedCount.fetch_add(1);
            }
        }
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

        // NO_BUFFERING: bypasses OS cache entirely — no dirty-page throttle stalls.
        // SEQUENTIAL_SCAN: hints to OS prefetcher (has no effect with NO_BUFFERING,
        // but harmless and useful if flags are ever changed).
        g_cameras[i].hFrames = CreateFileA(
            (path + "/frames.bin").c_str(),
            GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_NO_BUFFERING | FILE_FLAG_SEQUENTIAL_SCAN,
            nullptr);

        g_cameras[i].tsFile.open(path + "/timestamps.csv");
        g_cameras[i].tsFile << "saved_index,frame_index,timestamp_ticks\n";

        // Pre-allocate sector-aligned frame buffer pool
        g_cameras[i].bufs.reserve(CameraState::MAX_QUEUE_DEPTH);
        g_cameras[i].matPool.reserve(CameraState::MAX_QUEUE_DEPTH);
        for (int j = 0; j < CameraState::MAX_QUEUE_DEPTH; ++j)
        {
            auto* buf = static_cast<uint8_t*>(_aligned_malloc(FRAME_STRIDE, SECTOR));
            memset(buf + FRAME_BYTES, 0, FRAME_STRIDE - FRAME_BYTES); // zero padding
            g_cameras[i].bufs.push_back(buf);
            g_cameras[i].matPool.push_back(cv::Mat(IMG_H, IMG_W, CV_8UC1, buf));
            g_cameras[i].freeBufs.push_back(j);
        }

        // Metadata file for DIC software and playback
        std::ofstream meta(path + "/metadata.txt");
        meta << "width="        << IMG_W        << "\n"
             << "height="       << IMG_H        << "\n"
             << "channels=1\n"
             << "dtype=uint8\n"
             << "byte_order=little_endian\n"
             << "frame_stride=" << FRAME_STRIDE << "\n";
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
        std::cout << "Format: " << IMG_W << "x" << IMG_H
                  << " 8-bit mono, " << (FRAME_BYTES / 1024 / 1024.0)
                  << " MB/frame  stride=" << FRAME_STRIDE << " B" << std::endl;

        for (int i = 0; i < 4; ++i)
            g_cameras[i].writerThread = std::thread(writerThreadFunc, i);

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
                featureControls[i]->GetFloatFeature("AcquisitionFrameRate")->SetValue(70.0);
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
        std::cout << "Camera 1 started.\n" << std::endl;
        std::cout << "Recording... Press Q to stop.\n" << std::endl;

        while (true)
        {
            if (_kbhit())
            {
                char c = _getch();
                if (c == 'q' || c == 'Q')
                    break;
            }

            std::cout << "\r  Queue: ";
            for (int i = 0; i < 4; ++i)
            {
                std::lock_guard<std::mutex> lock(g_cameras[i].queueMtx);
                std::cout << "Cam" << (i + 1) << "=" << g_cameras[i].writeQueue.size() << "  ";
            }
            std::cout << std::flush;

            Sleep(500);
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

        std::cout << "Flushing write queues (waiting for disk)..." << std::endl;
        for (int i = 0; i < 4; ++i)
        {
            {
                std::lock_guard<std::mutex> lock(g_cameras[i].queueMtx);
                g_cameras[i].stopWriter.store(true);
                g_cameras[i].queueCv.notify_one();
            }
            g_cameras[i].writerThread.join();

            std::ofstream meta(g_cameras[i].savePath + "/metadata.txt", std::ios::app);
            meta << "frames=" << g_cameras[i].savedCount.load() << "\n";

            CloseHandle(g_cameras[i].hFrames);
            g_cameras[i].tsFile.close();

            for (auto* buf : g_cameras[i].bufs)
                _aligned_free(buf);
            g_cameras[i].bufs.clear();
            g_cameras[i].matPool.clear();
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
