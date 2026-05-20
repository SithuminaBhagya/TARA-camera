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

namespace fs = std::filesystem;

// index 0 = master (JCK26010021 / Camera 1), indices 1-3 = slaves
const std::vector<std::string> CAM_SNS = {
    "JCK26010021",  // Camera 1 — master
    "JCK26010020",  // Camera 2 — slave
    "JCK26010019",  // Camera 3 — slave
    "JCK26010013"   // Camera 4 — slave
};

const std::string SAVE_ROOT = "recordings";

// ── Frame item passed from callback to writer thread ──────────────
struct FrameItem
{
    cv::Mat  image;
    int      frameIndex;
    uint64_t timestamp;
};

// ── Per-camera state ──────────────────────────────────────────────
struct CameraState
{
    std::atomic<int>  frameCount{ 0 };    // frames received by callback
    std::atomic<int>  droppedCount{ 0 };  // frames dropped (queue full)
    std::string       savePath;
    std::ofstream     tsFile;

    std::queue<FrameItem>    writeQueue;
    std::mutex               queueMtx;
    std::condition_variable  queueCv;
    std::atomic<bool>        stopWriter{ false };
    std::thread              writerThread;

    // 16 frames × 16.8 MB = ~268 MB per camera buffer.
    // If "dropped > 0" in the final report, increase this or use faster storage.
    static const int MAX_QUEUE_DEPTH = 16;

    CameraState() = default;
    CameraState(const CameraState&) = delete;
    CameraState& operator=(const CameraState&) = delete;
};

CameraState g_cameras[4];

// ── Writer thread — one per camera ───────────────────────────────
// Runs independently of the SDK callback thread.
// Drains the queue completely before exiting so no frames are lost on shutdown.
void writerThreadFunc(int camIdx)
{
    auto& cam = g_cameras[camIdx];

    while (true)
    {
        FrameItem item;
        {
            std::unique_lock<std::mutex> lock(cam.queueMtx);
            cam.queueCv.wait(lock, [&]{
                return !cam.writeQueue.empty() || cam.stopWriter.load();
            });

            if (cam.writeQueue.empty())
                break; // stop requested and queue fully drained

            item = std::move(cam.writeQueue.front());
            cam.writeQueue.pop();
        }

        // Write JPEG
        std::ostringstream ss;
        ss << cam.savePath << "/frame_"
           << std::setw(6) << std::setfill('0') << item.frameIndex << ".jpg";
        cv::imwrite(ss.str(), item.image, { cv::IMWRITE_JPEG_QUALITY, 95 });

        // Write timestamp
        cam.tsFile << item.frameIndex << "," << item.timestamp << "\n";
    }
}

// ── Callback — runs on SDK thread, must return fast ──────────────
class CCaptureHandler : public ICaptureEventHandler
{
public:
    int m_index;
    CCaptureHandler(int index) : m_index(index) {}

    void DoOnImageCaptured(CImageDataPointer& imgPtr, void* pUserParam) override
    {
        if (imgPtr->GetStatus() != GX_FRAME_STATUS_SUCCESS)
            return;

        int width    = (int)imgPtr->GetWidth();
        int height   = (int)imgPtr->GetHeight();
        uint64_t ts  = imgPtr->GetTimeStamp();

        void* pRaw8 = imgPtr->ConvertToRaw8(GX_BIT_0_7);
        cv::Mat grayMat(height, width, CV_8UC1, pRaw8);
        cv::Mat bgrMat;
        cv::cvtColor(grayMat, bgrMat, cv::COLOR_GRAY2BGR);

        auto& cam   = g_cameras[m_index];
        int   count = cam.frameCount.fetch_add(1);

        // Push to writer queue — never block here
        {
            std::lock_guard<std::mutex> lock(cam.queueMtx);
            if ((int)cam.writeQueue.size() < CameraState::MAX_QUEUE_DEPTH)
            {
                cam.writeQueue.push({ bgrMat.clone(), count, ts });
                cam.queueCv.notify_one();
            }
            else
            {
                cam.droppedCount.fetch_add(1);
            }
        }
    }
};

// ── Folder creation ───────────────────────────────────────────────
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
        g_cameras[i].tsFile.open(path + "/timestamps.csv");
        g_cameras[i].tsFile << "frame_index,timestamp_ticks\n";
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

        // Start writer threads before acquisition begins
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

            featureControls[i]->GetIntFeature("Width")->SetValue(2600);
            featureControls[i]->GetIntFeature("Height")->SetValue(2160);
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
            std::cout << "Camera " << (i + 1) << " (" << CAM_SNS[i] << ", " << role << ") configured." << std::endl;
        }

        // Arm all buffer queues
        for (int i = 0; i < 4; ++i)
            streams[i]->StartGrab();

        // Start slaves first — they wait for trigger
        for (int i = 1; i < 4; ++i)
        {
            featureControls[i]->GetCommandFeature("AcquisitionStart")->Execute();
            std::cout << "Camera " << (i + 1) << " (" << CAM_SNS[i] << ") armed." << std::endl;
        }

        // Start master last — begins sending trigger pulses
        featureControls[0]->GetCommandFeature("AcquisitionStart")->Execute();
        std::cout << "Camera 1 (" << CAM_SNS[0] << ") started." << std::endl;
        std::cout << "\nRecording... Press Q to stop.\n" << std::endl;

        while (true)
        {
            if (_kbhit())
            {
                char c = _getch();
                if (c == 'q' || c == 'Q')
                    break;
            }

            // Print live queue depths so you can see if disk is keeping up
            std::cout << "\r  Queue: ";
            for (int i = 0; i < 4; ++i)
            {
                std::lock_guard<std::mutex> lock(g_cameras[i].queueMtx);
                std::cout << "Cam" << (i + 1) << "=" << g_cameras[i].writeQueue.size() << "  ";
            }
            std::cout << std::flush;

            Sleep(500);
        }

        // Stop master first — no more trigger pulses
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

        // Signal writer threads to finish draining then exit
        std::cout << "Flushing write queues (waiting for disk)..." << std::endl;
        for (int i = 0; i < 4; ++i)
        {
            {
                std::lock_guard<std::mutex> lock(g_cameras[i].queueMtx);
                g_cameras[i].stopWriter.store(true);
                g_cameras[i].queueCv.notify_one();
            }
            g_cameras[i].writerThread.join();
            g_cameras[i].tsFile.close();
        }

        IGXFactory::GetInstance().Uninit();

        std::cout << "\nResults:" << std::endl;
        for (int i = 0; i < 4; ++i)
        {
            std::string role = (i == 0) ? "master" : "slave ";
            int captured = g_cameras[i].frameCount.load();
            int dropped  = g_cameras[i].droppedCount.load();
            std::cout << "  Camera " << (i + 1) << " (" << role << " " << CAM_SNS[i] << "): "
                      << captured << " captured";
            if (dropped > 0)
                std::cout << ",  " << dropped << " DROPPED (queue full — disk too slow)";
            std::cout << std::endl;
        }
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
