#include "GalaxyIncludes.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <conio.h>

namespace fs = std::filesystem;

// index 0 = master (JCK26010021 / SN 10021), indices 1-3 = slaves
const std::vector<std::string> CAM_SNS = {
    "JCK26010021",  // 0 — master
    "JCK26010020",  // 1 — slave
    "JCK26010019",  // 2 — slave
    "JCK26010013"   // 3 — slave
};

const std::string SAVE_ROOT = "recordings";

struct CameraState
{
    std::atomic<int> frameCount{ 0 };
    std::string savePath;
    std::ofstream tsFile;

    CameraState() = default;
    CameraState(const CameraState&) = delete;
    CameraState& operator=(const CameraState&) = delete;
};

CameraState g_cameras[4];

// ── Callback ──────────────────────────────────────────────────────
class CCaptureHandler : public ICaptureEventHandler
{
public:
    int m_index;
    CCaptureHandler(int index) : m_index(index) {}

    void DoOnImageCaptured(CImageDataPointer& imgPtr, void* pUserParam) override
    {
        if (imgPtr->GetStatus() != GX_FRAME_STATUS_SUCCESS)
            return;

        int width  = (int)imgPtr->GetWidth();
        int height = (int)imgPtr->GetHeight();
        uint64_t ts = imgPtr->GetTimestamp();

        void* pRaw8 = imgPtr->ConvertToRaw8(GX_BIT_0_7);
        cv::Mat grayMat(height, width, CV_8UC1, pRaw8);
        cv::Mat bgrMat;
        cv::cvtColor(grayMat, bgrMat, cv::COLOR_GRAY2BGR);

        int count = g_cameras[m_index].frameCount.fetch_add(1);

        std::ostringstream ss;
        ss << g_cameras[m_index].savePath
           << "/frame_" << std::setw(6) << std::setfill('0') << count << ".jpg";
        cv::imwrite(ss.str(), bgrMat, { cv::IMWRITE_JPEG_QUALITY, 95 });

        // one callback thread per camera — no concurrent writes to this file
        g_cameras[m_index].tsFile << count << "," << ts << "\n";
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
        std::string path = expPath + "/camera_" + std::to_string(i);
        fs::create_directories(path);
        g_cameras[i].savePath = path;
        g_cameras[i].tsFile.open(path + "/timestamps.csv");
        g_cameras[i].tsFile << "frame_index,timestamp_ticks\n";
    }

    return expPath;
}

// ── Trigger configuration ─────────────────────────────────────────
// Master: free-runs, outputs ExposureActive pulse on Line1.
// Slave : waits for rising edge on Line0 before each capture.
// Wiring: master Line1 (output) → slave Line0 (input) on all 3 slaves.

void configureMaster(CGXFeatureControlPointer& fc)
{
    fc->GetEnumFeature("TriggerMode")->SetValue("Off");
    fc->GetEnumFeature("LineSelector")->SetValue("Line1");
    fc->GetEnumFeature("LineMode")->SetValue("Output");
    fc->GetEnumFeature("LineSource")->SetValue("ExposureActive");
}

void configureSlave(CGXFeatureControlPointer& fc)
{
    fc->GetEnumFeature("TriggerMode")->SetValue("On");
    fc->GetEnumFeature("TriggerSource")->SetValue("Line0");
    fc->GetEnumFeature("TriggerActivation")->SetValue("RisingEdge");
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
                configureMaster(featureControls[i]);
            else
                configureSlave(featureControls[i]);

            streams[i] = cameras[i]->OpenStream(0);
            callbacks[i] = std::make_shared<CCaptureHandler>(i);
            streams[i]->RegisterCaptureCallback(callbacks[i].get(), nullptr);

            std::string role = (i == 0) ? "master" : "slave";
            std::cout << "Camera " << i << " (" << CAM_SNS[i] << ", " << role << ") configured." << std::endl;
        }

        // Phase 1: Arm all buffer queues
        for (int i = 0; i < 4; ++i)
            streams[i]->StartGrab();

        // Phase 2: Start slaves first — they block waiting for trigger
        for (int i = 1; i < 4; ++i)
        {
            featureControls[i]->GetCommandFeature("AcquisitionStart")->Execute();
            std::cout << "Slave " << i << " (" << CAM_SNS[i] << ") armed." << std::endl;
        }

        // Phase 3: Start master last — it begins sending trigger pulses
        featureControls[0]->GetCommandFeature("AcquisitionStart")->Execute();
        std::cout << "Master (" << CAM_SNS[0] << ") started." << std::endl;
        std::cout << "\nRecording... Press Q to stop.\n" << std::endl;

        while (true)
        {
            if (_kbhit())
            {
                char c = _getch();
                if (c == 'q' || c == 'Q')
                    break;
            }
            Sleep(10);
        }

        // Stop master first — no more trigger pulses
        std::cout << "\nShutting down..." << std::endl;
        featureControls[0]->GetCommandFeature("AcquisitionStop")->Execute();
        for (int i = 1; i < 4; ++i)
            featureControls[i]->GetCommandFeature("AcquisitionStop")->Execute();

        for (int i = 0; i < 4; ++i)
        {
            streams[i]->StopGrab();
            streams[i]->UnregisterCaptureCallback();
            streams[i]->Close();
            cameras[i]->Close();
            g_cameras[i].tsFile.close();
        }

        IGXFactory::GetInstance().Uninit();

        std::cout << "\nFrames saved:" << std::endl;
        for (int i = 0; i < 4; ++i)
        {
            std::string role = (i == 0) ? "master" : "slave ";
            std::cout << "  Cam " << i << " (" << role << " " << CAM_SNS[i] << "): "
                      << g_cameras[i].frameCount << " frames" << std::endl;
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
