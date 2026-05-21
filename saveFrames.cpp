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
const int    BATCH_SIZE    = 10;   // frames per WriteFile call per camera (0.67 s at 15 fps)

// ── Frame item passed from callback to writer thread ──────────────
struct FrameItem
{
    int      camIdx;
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
    // freeBufs   — indices of buffers not currently in the global write queue
    std::vector<uint8_t*> bufs;
    std::vector<cv::Mat>  matPool;
    std::vector<int>      freeBufs;
    std::mutex            freeBufsMtx;

    // Batch write buffer — BATCH_SIZE frames assembled here, flushed in one WriteFile call
    uint8_t*  batchBuf{ nullptr };
    int       batchFillCount{ 0 };
    FrameItem batchItems[BATCH_SIZE];   // metadata for frames waiting in batchBuf

    // 256 frames × 5.6 MB = ~1.44 GB per camera (~5.8 GB total for 4 cameras).
    // Covers worst-case 17s OS write stall at 15fps (15 × 11 = 165 frames per camera observed max).
    static const int MAX_QUEUE_DEPTH = 256;

    CameraState() = default;
    CameraState(const CameraState&) = delete;
    CameraState& operator=(const CameraState&) = delete;
};

CameraState g_cameras[4];

// ── Single writer thread — avoids concurrent page-cache lock contention ──
std::deque<FrameItem>    g_writeQueue;
std::mutex               g_queueMtx;
std::condition_variable  g_queueCv;
std::atomic<bool>        g_stopWriter{ false };
std::thread              g_writerThread;

void writerThreadFunc()
{
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);

    double totalWriteMs[4] = {};
    double maxWriteMs[4]   = {};
    int    writeCount[4]   = {};

    auto flushBatch = [&](int ci)
    {
        auto& cam = g_cameras[ci];
        if (cam.batchFillCount == 0) return;

        DWORD  written = 0;
        size_t nBytes  = (size_t)cam.batchFillCount * FRAME_STRIDE;
        auto   t0      = std::chrono::steady_clock::now();
        WriteFile(cam.hFrames, cam.batchBuf, (DWORD)nBytes, &written, nullptr);
        double ms = std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - t0).count();

        totalWriteMs[ci] += ms;
        if (ms > maxWriteMs[ci]) maxWriteMs[ci] = ms;
        writeCount[ci]++;

        for (int k = 0; k < cam.batchFillCount; ++k)
        {
            int savedIdx = cam.savedCount.fetch_add(1);
            cam.tsFile << savedIdx << ","
                       << cam.batchItems[k].frameIndex << ","
                       << cam.batchItems[k].timestamp  << "\n";
        }
        cam.batchFillCount = 0;
    };

    while (true)
    {
        FrameItem item;
        {
            std::unique_lock<std::mutex> lk(g_queueMtx);
            g_queueCv.wait(lk, []{
                return !g_writeQueue.empty() || g_stopWriter.load();
            });
            if (g_writeQueue.empty())
                break;
            item = g_writeQueue.front();
            g_writeQueue.pop_front();
        }

        int   ci  = item.camIdx;
        auto& cam = g_cameras[ci];

        memcpy(cam.batchBuf + (size_t)cam.batchFillCount * FRAME_STRIDE,
               cam.bufs[item.bufIdx], FRAME_BYTES);
        cam.batchItems[cam.batchFillCount] = item;
        cam.batchFillCount++;

        {
            std::lock_guard<std::mutex> lk(cam.freeBufsMtx);
            cam.freeBufs.push_back(item.bufIdx);
        }

        if (cam.batchFillCount == BATCH_SIZE)
            flushBatch(ci);
    }

    for (int i = 0; i < 4; ++i) flushBatch(i);
    for (int i = 0; i < 4; ++i) { FlushFileBuffers(g_cameras[i].hFrames); g_cameras[i].tsFile.flush(); }

    for (int i = 0; i < 4; ++i)
    {
        if (writeCount[i] > 0)
        {
            double avgMs   = totalWriteMs[i] / writeCount[i];
            double batchMB = (double)BATCH_SIZE * FRAME_STRIDE / 1024.0 / 1024.0;
            double mbps    = batchMB / (avgMs / 1000.0);
            std::cout << "  Cam" << (i + 1) << " write: avg " << std::fixed
                      << std::setprecision(2) << avgMs << " ms  max " << maxWriteMs[i]
                      << " ms  (" << (int)mbps << " MB/s)"
                      << "  [" << BATCH_SIZE << " frames/write]" << std::endl;
        }
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

        g_cameras[i].hFrames = CreateFileA(
            (path + "/frames.bin").c_str(),
            GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
            FILE_FLAG_NO_BUFFERING | FILE_FLAG_SEQUENTIAL_SCAN,
            nullptr);

        g_cameras[i].batchBuf = static_cast<uint8_t*>(
            _aligned_malloc((size_t)BATCH_SIZE * FRAME_STRIDE, SECTOR));
        memset(g_cameras[i].batchBuf, 0, (size_t)BATCH_SIZE * FRAME_STRIDE);

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

            size_t qDepth;
            {
                std::lock_guard<std::mutex> lock(g_queueMtx);
                qDepth = g_writeQueue.size();
            }
            std::cout << "\r  Queue: " << qDepth << " pending  " << std::flush;

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

            _aligned_free(g_cameras[i].batchBuf);
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
