#include "GalaxyIncludes.h"
#include <opencv2/opencv.hpp>
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

// PNG compression level: 1 = fastest (worst ratio), 9 = slowest (best ratio).
// Level 1 typical: ~50-100 ms/frame, ratio ~1.5-2x for natural images.
const int PNG_COMPRESSION_LEVEL = 1;

// ── Item passed from callback to per-camera compression thread ─────
struct FrameItem
{
    int      camIdx;
    int      bufIdx;
    int      frameIndex;
    uint64_t timestamp;
};

// ── Item passed from compression thread to global writer ───────────
struct CompressedItem
{
    int                camIdx;
    int                frameIndex;
    uint64_t           timestamp;
    std::vector<uchar> data;        // PNG-encoded bytes
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

    // Raw frame buffer pool — uncompressed FRAME_BYTES each.
    // Callback grabs a free bufIdx, memcpy's raw pixels in.
    // Compression thread reads from bufs[bufIdx], then returns bufIdx to freeBufs.
    std::vector<uint8_t*> bufs;
    std::vector<int>      freeBufs;
    std::mutex            freeBufsMtx;

    // Per-camera compression queue (raw frames waiting to be encoded).
    std::deque<FrameItem>    compressQueue;
    std::mutex               compressMtx;
    std::condition_variable  compressCv;
    std::thread              compressThread;

    // 128 frames × 5.4 MB raw = ~690 MB per camera (~2.8 GB total for 4 cameras).
    // Compression keeps up with cameras; this is just a small absorption buffer.
    static const int MAX_QUEUE_DEPTH = 128;

    CameraState() = default;
    CameraState(const CameraState&) = delete;
    CameraState& operator=(const CameraState&) = delete;
};

CameraState g_cameras[4];

// ── Global write queue (compressed frames from all cameras) ────────
std::deque<CompressedItem>  g_writeQueue;
std::mutex                  g_queueMtx;
std::condition_variable     g_queueCv;
std::atomic<bool>           g_stopWriter{ false };
std::atomic<bool>           g_stopCompress{ false };
std::thread                 g_writerThread;

// ── Per-camera compression thread ──────────────────────────────────
void compressionThreadFunc(int camIdx)
{
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    auto& cam = g_cameras[camIdx];

    std::vector<int> pngParams = { cv::IMWRITE_PNG_COMPRESSION, PNG_COMPRESSION_LEVEL };

    double totalCompMs = 0.0;
    double maxCompMs   = 0.0;
    size_t totalBytes  = 0;
    int    compCount   = 0;

    while (true)
    {
        FrameItem item;
        {
            std::unique_lock<std::mutex> lk(cam.compressMtx);
            cam.compressCv.wait(lk, [&] {
                return !cam.compressQueue.empty() || g_stopCompress.load();
            });
            if (cam.compressQueue.empty())
                break;
            item = cam.compressQueue.front();
            cam.compressQueue.pop_front();
        }

        // Wrap raw buffer in cv::Mat header (no copy)
        cv::Mat raw(IMG_H, IMG_W, CV_8UC1, cam.bufs[item.bufIdx]);

        CompressedItem out;
        out.camIdx     = item.camIdx;
        out.frameIndex = item.frameIndex;
        out.timestamp  = item.timestamp;

        auto t0 = std::chrono::steady_clock::now();
        cv::imencode(".png", raw, out.data, pngParams);
        double ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0).count();
        totalCompMs += ms;
        if (ms > maxCompMs) maxCompMs = ms;
        totalBytes += out.data.size();
        compCount++;

        // Release raw buffer back to camera's free pool
        {
            std::lock_guard<std::mutex> lk(cam.freeBufsMtx);
            cam.freeBufs.push_back(item.bufIdx);
        }

        // Hand compressed frame to the writer
        {
            std::lock_guard<std::mutex> lk(g_queueMtx);
            g_writeQueue.push_back(std::move(out));
        }
        g_queueCv.notify_one();
    }

    if (compCount > 0)
    {
        double avgMs = totalCompMs / compCount;
        double avgKB = (double)totalBytes / compCount / 1024.0;
        double ratio = (double)FRAME_BYTES / ((double)totalBytes / compCount);
        std::cout << "  Cam" << (camIdx + 1) << " compress: avg " << std::fixed
                  << std::setprecision(2) << avgMs << " ms  max " << maxCompMs
                  << " ms  avg " << std::setprecision(1) << avgKB << " KB/frame  "
                  << std::setprecision(2) << ratio << "x ratio" << std::endl;
    }
}

// ── Single writer thread — variable-size compressed writes ──────────
void writerThreadFunc()
{
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    double totalWriteMs[4] = {};
    double maxWriteMs[4]   = {};
    size_t totalBytes[4]   = {};
    int    writeCount[4]   = {};

    while (true)
    {
        CompressedItem item;
        {
            std::unique_lock<std::mutex> lk(g_queueMtx);
            g_queueCv.wait(lk, [] {
                return !g_writeQueue.empty() || g_stopWriter.load();
            });
            if (g_writeQueue.empty())
                break;
            item = std::move(g_writeQueue.front());
            g_writeQueue.pop_front();
        }

        int   ci  = item.camIdx;
        auto& cam = g_cameras[ci];

        uint32_t size = (uint32_t)item.data.size();
        DWORD    written = 0;

        auto t0 = std::chrono::steady_clock::now();
        WriteFile(cam.hFrames, &size, sizeof(size), &written, nullptr);
        WriteFile(cam.hFrames, item.data.data(), size, &written, nullptr);
        double ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0).count();

        totalWriteMs[ci] += ms;
        if (ms > maxWriteMs[ci]) maxWriteMs[ci] = ms;
        totalBytes[ci] += size + sizeof(size);
        writeCount[ci]++;

        int savedIdx = cam.savedCount.fetch_add(1);
        cam.tsFile << savedIdx << ","
                   << item.frameIndex << ","
                   << item.timestamp  << "\n";
    }

    for (int i = 0; i < 4; ++i)
    {
        FlushFileBuffers(g_cameras[i].hFrames);
        g_cameras[i].tsFile.flush();
    }

    for (int i = 0; i < 4; ++i)
    {
        if (writeCount[i] > 0)
        {
            double avgMs   = totalWriteMs[i] / writeCount[i];
            double totalMB = (double)totalBytes[i] / 1024.0 / 1024.0;
            std::cout << "  Cam" << (i + 1) << " write:    avg " << std::fixed
                      << std::setprecision(2) << avgMs << " ms  max " << maxWriteMs[i]
                      << " ms  total " << std::setprecision(1) << totalMB << " MB" << std::endl;
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

        {
            std::lock_guard<std::mutex> lk(cam.compressMtx);
            cam.compressQueue.push_back({ m_index, bufIdx, count, ts });
        }
        cam.compressCv.notify_one();
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

        // Buffered writes — compressed data rate is low enough to avoid
        // the dirty-page throttle that bit us with raw frames.
        g_cameras[i].hFrames = CreateFileA(
            (path + "/frames.bin").c_str(),
            GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
            FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN,
            nullptr);

        g_cameras[i].tsFile.open(path + "/timestamps.csv");
        g_cameras[i].tsFile << "saved_index,frame_index,timestamp_ticks\n";

        // Raw frame buffer pool — plain malloc, no alignment needed for PNG path
        g_cameras[i].bufs.reserve(CameraState::MAX_QUEUE_DEPTH);
        for (int j = 0; j < CameraState::MAX_QUEUE_DEPTH; ++j)
        {
            auto* buf = static_cast<uint8_t*>(malloc(FRAME_BYTES));
            g_cameras[i].bufs.push_back(buf);
            g_cameras[i].freeBufs.push_back(j);
        }

        // Metadata: format=png signals variable-size [u32 size][data] records
        std::ofstream meta(path + "/metadata.txt");
        meta << "width="    << IMG_W << "\n"
             << "height="   << IMG_H << "\n"
             << "channels=1\n"
             << "dtype=uint8\n"
             << "format=png\n"
             << "png_level=" << PNG_COMPRESSION_LEVEL << "\n";
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
                  << " MB/frame raw  →  PNG level " << PNG_COMPRESSION_LEVEL << std::endl;

        // Start writer first, then compression threads
        g_writerThread = std::thread(writerThreadFunc);
        for (int i = 0; i < 4; ++i)
            g_cameras[i].compressThread = std::thread(compressionThreadFunc, i);

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
                featureControls[i]->GetFloatFeature("AcquisitionFrameRate")->SetValue(15.0);
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

            size_t writeQ;
            size_t compQ[4];
            {
                std::lock_guard<std::mutex> lock(g_queueMtx);
                writeQ = g_writeQueue.size();
            }
            for (int i = 0; i < 4; ++i)
            {
                std::lock_guard<std::mutex> lk(g_cameras[i].compressMtx);
                compQ[i] = g_cameras[i].compressQueue.size();
            }
            std::cout << "\r  Compress: ["
                      << compQ[0] << "/" << compQ[1] << "/"
                      << compQ[2] << "/" << compQ[3] << "]   Write: "
                      << writeQ << "          " << std::flush;

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

        std::cout << "Draining compression queues..." << std::endl;
        g_stopCompress.store(true);
        for (int i = 0; i < 4; ++i)
        {
            g_cameras[i].compressCv.notify_one();
            g_cameras[i].compressThread.join();
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
            std::ofstream meta(g_cameras[i].savePath + "/metadata.txt", std::ios::app);
            meta << "frames=" << g_cameras[i].savedCount.load() << "\n";

            CloseHandle(g_cameras[i].hFrames);
            g_cameras[i].tsFile.close();

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
