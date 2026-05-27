#include "GalaxyIncludes.h"
#include <zstd.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <atomic>
#include <deque>
#include <map>
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
#include <cstring>

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

// Test configuration: 70 fps Mono8 with Zstd compression for 100 seconds.
//
// Compression budget per Zstd-1 benchmarks: ~10-15 ms per frame, so
// 2 compress threads/cam × ~67 fps/thread ≈ 133 fps per camera capacity
// vs 70 fps demand → ~90% headroom.  16 PNG threads worth of work
// is replaced with 8 Zstd threads.
const int    ZSTD_COMPRESSION_LEVEL    = 1;
const int    FRAMES_PER_CHUNK          = 200;     // ~2.9 sec per chunk at 70 fps
const int    COMPRESS_THREADS_PER_CAM  = 2;
const double CAPTURE_FPS               = 70.0;
const int    RECORD_SECONDS            = 100;

// ── Item passed from callback to per-camera compression thread ─────
struct FrameItem
{
    int      camIdx;
    int      bufIdx;
    int      frameIndex;   // contiguous within a camera; gap-free
    uint64_t timestamp;
};

// ── Item passed from compression thread to global writer ───────────
struct CompressedItem
{
    int                camIdx;
    int                frameIndex;
    uint64_t           timestamp;
    std::vector<uint8_t> data;       // Zstd-compressed bytes
};

// ── Per-camera state ──────────────────────────────────────────────
struct CameraState
{
    std::atomic<int>  frameCount  { 0 };   // assigned after buffer grab
    std::atomic<int>  savedCount  { 0 };
    std::atomic<int>  droppedCount{ 0 };
    std::string       savePath;

    HANDLE        hFrames{ INVALID_HANDLE_VALUE };
    std::ofstream tsFile;

    // Chunk rotation state (writer thread only)
    int currentChunkIndex   { 0 };
    int framesInCurrentChunk{ 0 };

    // Raw frame buffer pool — uncompressed FRAME_BYTES each.
    std::vector<uint8_t*> bufs;
    std::vector<int>      freeBufs;
    std::mutex            freeBufsMtx;

    // Per-camera compression queue (workers drain it)
    std::deque<FrameItem>     compressQueue;
    std::mutex                compressMtx;
    std::condition_variable   compressCv;
    std::vector<std::thread>  compressThreads;

    // Per-camera compression stats (atomic — many writers)
    std::atomic<int64_t>      totalCompressUs   { 0 };
    std::atomic<int>          compressedCount   { 0 };
    std::atomic<int64_t>      totalCompressBytes{ 0 };
    std::mutex                maxCompressMtx;
    double                    maxCompressMs     { 0.0 };

    // Writer-side reorder buffer (multiple compress threads can complete
    // frames out of order; writer holds future frames until predecessors
    // arrive, then drains in order).
    std::map<int, CompressedItem> reorderBuf;
    int                            nextExpectedFrameIdx{ 0 };

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

// Background close threads (offload CloseHandle, which can stall on flush)
std::vector<std::thread>    g_closeThreads;
std::mutex                  g_closeThreadsMtx;

// ── Chunk filename helpers ────────────────────────────────────────
std::string chunkPath(const std::string& camFolder, int chunkIdx)
{
    std::ostringstream ss;
    ss << camFolder << "/chunk_"
       << std::setw(3) << std::setfill('0') << chunkIdx << ".bin";
    return ss.str();
}

HANDLE openChunkFile(int camIdx, int chunkIdx)
{
    return CreateFileA(
        chunkPath(g_cameras[camIdx].savePath, chunkIdx).c_str(),
        GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
        FILE_ATTRIBUTE_NORMAL | FILE_FLAG_SEQUENTIAL_SCAN,
        nullptr);
}

// ── Compression worker (multiple per camera) ───────────────────────
void compressionThreadFunc(int camIdx)
{
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    auto& cam = g_cameras[camIdx];

    // Per-thread Zstd context — reused across all frames this thread handles.
    // Avoids per-frame allocation/init overhead.
    ZSTD_CCtx* cctx = ZSTD_createCCtx();
    ZSTD_CCtx_setParameter(cctx, ZSTD_c_compressionLevel, ZSTD_COMPRESSION_LEVEL);

    // Worst-case output buffer size for one frame
    const size_t bound = ZSTD_compressBound(FRAME_BYTES);
    std::vector<uint8_t> outBuf(bound);

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

        auto t0 = std::chrono::steady_clock::now();
        size_t compressedSize = ZSTD_compress2(cctx,
            outBuf.data(), bound,
            cam.bufs[item.bufIdx], FRAME_BYTES);
        auto dur = std::chrono::steady_clock::now() - t0;

        if (ZSTD_isError(compressedSize))
        {
            // Compression failure — extremely unlikely with stock Zstd,
            // but treat the frame as dropped if it happens.
            cam.droppedCount.fetch_add(1);
            {
                std::lock_guard<std::mutex> lk(cam.freeBufsMtx);
                cam.freeBufs.push_back(item.bufIdx);
            }
            continue;
        }

        int64_t us = std::chrono::duration_cast<std::chrono::microseconds>(dur).count();
        double  ms = us / 1000.0;

        CompressedItem out;
        out.camIdx     = item.camIdx;
        out.frameIndex = item.frameIndex;
        out.timestamp  = item.timestamp;
        out.data.assign(outBuf.begin(), outBuf.begin() + compressedSize);

        cam.totalCompressUs.fetch_add(us);
        cam.compressedCount.fetch_add(1);
        cam.totalCompressBytes.fetch_add((int64_t)compressedSize);
        {
            std::lock_guard<std::mutex> lk(cam.maxCompressMtx);
            if (ms > cam.maxCompressMs) cam.maxCompressMs = ms;
        }

        // Return raw buffer to camera's free pool
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

    ZSTD_freeCCtx(cctx);
}

// ── Helper: actually write one CompressedItem to disk ─────────────
static void writeCompressedFrame(int ci, CompressedItem& item,
                                 double totalMsArr[4], double maxMsArr[4],
                                 size_t totalBytesArr[4], int writeCountArr[4])
{
    auto& cam = g_cameras[ci];

    uint32_t size = (uint32_t)item.data.size();
    DWORD    written = 0;

    auto t0 = std::chrono::steady_clock::now();
    WriteFile(cam.hFrames, &size, sizeof(size), &written, nullptr);
    WriteFile(cam.hFrames, item.data.data(), size, &written, nullptr);
    double ms = std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - t0).count();

    totalMsArr[ci]    += ms;
    if (ms > maxMsArr[ci]) maxMsArr[ci] = ms;
    totalBytesArr[ci] += size + sizeof(size);
    writeCountArr[ci]++;

    int savedIdx = cam.savedCount.fetch_add(1);
    cam.tsFile << savedIdx << ","
               << item.frameIndex << ","
               << item.timestamp  << "\n";

    cam.framesInCurrentChunk++;
    if (cam.framesInCurrentChunk >= FRAMES_PER_CHUNK)
    {
        HANDLE oldHandle = cam.hFrames;
        cam.currentChunkIndex++;
        cam.framesInCurrentChunk = 0;
        cam.hFrames = openChunkFile(ci, cam.currentChunkIndex);

        std::lock_guard<std::mutex> lk(g_closeThreadsMtx);
        g_closeThreads.emplace_back([oldHandle]() { CloseHandle(oldHandle); });
    }

    cam.nextExpectedFrameIdx++;
}

// ── Single writer thread — reorder + chunk rotation ────────────────
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

        if (item.frameIndex == cam.nextExpectedFrameIdx)
        {
            writeCompressedFrame(ci, item,
                                 totalWriteMs, maxWriteMs, totalBytes, writeCount);

            while (!cam.reorderBuf.empty())
            {
                auto it = cam.reorderBuf.begin();
                if (it->first != cam.nextExpectedFrameIdx) break;
                writeCompressedFrame(ci, it->second,
                                     totalWriteMs, maxWriteMs, totalBytes, writeCount);
                cam.reorderBuf.erase(it);
            }
        }
        else if (item.frameIndex > cam.nextExpectedFrameIdx)
        {
            cam.reorderBuf.emplace(item.frameIndex, std::move(item));
        }
    }

    // Drain any remaining in-order frames from reorder buffers
    for (int ci = 0; ci < 4; ++ci)
    {
        auto& cam = g_cameras[ci];
        while (!cam.reorderBuf.empty())
        {
            auto it = cam.reorderBuf.begin();
            if (it->first != cam.nextExpectedFrameIdx) break;
            writeCompressedFrame(ci, it->second,
                                 totalWriteMs, maxWriteMs, totalBytes, writeCount);
            cam.reorderBuf.erase(it);
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

        auto& cam = g_cameras[m_index];

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

        int count = cam.frameCount.fetch_add(1);

        std::memcpy(cam.bufs[bufIdx], pRaw8, FRAME_BYTES);

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

        g_cameras[i].currentChunkIndex     = 0;
        g_cameras[i].framesInCurrentChunk  = 0;
        g_cameras[i].nextExpectedFrameIdx  = 0;
        g_cameras[i].hFrames = openChunkFile(i, 0);

        g_cameras[i].tsFile.open(path + "/timestamps.csv");
        g_cameras[i].tsFile << "saved_index,frame_index,timestamp_ticks\n";

        g_cameras[i].bufs.reserve(CameraState::MAX_QUEUE_DEPTH);
        for (int j = 0; j < CameraState::MAX_QUEUE_DEPTH; ++j)
        {
            auto* buf = static_cast<uint8_t*>(malloc(FRAME_BYTES));
            g_cameras[i].bufs.push_back(buf);
            g_cameras[i].freeBufs.push_back(j);
        }

        // Metadata format: each chunk is [u32 size][zstd bytes] records.
        // Decompresses back to FRAME_BYTES of raw Mono8 pixel data.
        std::ofstream meta(path + "/metadata.txt");
        meta << "width="             << IMG_W                    << "\n"
             << "height="            << IMG_H                    << "\n"
             << "channels=1\n"
             << "dtype=uint8\n"
             << "format=zstd\n"
             << "zstd_level="        << ZSTD_COMPRESSION_LEVEL   << "\n"
             << "frames_per_chunk="  << FRAMES_PER_CHUNK         << "\n"
             << "fps="               << CAPTURE_FPS              << "\n"
             << "compress_threads="  << COMPRESS_THREADS_PER_CAM << "\n";
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
        std::cout << "Mode: " << IMG_W << "x" << IMG_H
                  << " 8-bit mono ZSTD (level " << ZSTD_COMPRESSION_LEVEL
                  << ") @ " << CAPTURE_FPS << " fps for " << RECORD_SECONDS
                  << " sec  (" << COMPRESS_THREADS_PER_CAM
                  << " compress threads/cam, " << (COMPRESS_THREADS_PER_CAM * 4)
                  << " total)\n" << std::endl;

        g_writerThread = std::thread(writerThreadFunc);
        for (int i = 0; i < 4; ++i)
        {
            g_cameras[i].compressThreads.reserve(COMPRESS_THREADS_PER_CAM);
            for (int t = 0; t < COMPRESS_THREADS_PER_CAM; ++t)
                g_cameras[i].compressThreads.emplace_back(compressionThreadFunc, i);
        }

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
            std::cout << "\r  t=" << std::fixed << std::setprecision(1)
                      << (elapsed / 1000.0) << "s  Compress: ["
                      << compQ[0] << "/" << compQ[1] << "/"
                      << compQ[2] << "/" << compQ[3] << "]   Write: "
                      << writeQ << "          " << std::flush;

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

        std::cout << "Draining compression queues..." << std::endl;
        g_stopCompress.store(true);
        for (int i = 0; i < 4; ++i)
        {
            g_cameras[i].compressCv.notify_all();
            for (auto& t : g_cameras[i].compressThreads)
                if (t.joinable()) t.join();
        }

        // Print compression stats
        for (int i = 0; i < 4; ++i)
        {
            auto& cam = g_cameras[i];
            int n = cam.compressedCount.load();
            if (n > 0)
            {
                double avgMs   = (cam.totalCompressUs.load() / 1000.0) / n;
                double avgKB   = (double)cam.totalCompressBytes.load() / n / 1024.0;
                double ratio   = (double)FRAME_BYTES /
                                 ((double)cam.totalCompressBytes.load() / n);
                std::cout << "  Cam" << (i + 1) << " compress: avg " << std::fixed
                          << std::setprecision(2) << avgMs << " ms  max "
                          << cam.maxCompressMs << " ms  avg "
                          << std::setprecision(1) << avgKB << " KB/frame  "
                          << std::setprecision(2) << ratio << "x ratio  ("
                          << n << " frames)" << std::endl;
            }
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
            meta << "frames=" << g_cameras[i].savedCount.load()       << "\n"
                 << "chunks=" << (g_cameras[i].currentChunkIndex + 1) << "\n";

            for (auto* buf : g_cameras[i].bufs)
                free(buf);
            g_cameras[i].bufs.clear();
        }

        std::cout << "Waiting for background flushes (" << g_closeThreads.size()
                  << " chunks)..." << std::endl;
        for (auto& t : g_closeThreads)
            if (t.joinable()) t.join();
        g_closeThreads.clear();

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
