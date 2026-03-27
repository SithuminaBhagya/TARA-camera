#include "GalaxyIncludes.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <atomic>
#include <sstream>
#include <iomanip>
#include <filesystem>
#include <conio.h>

namespace fs = std::filesystem;

// ── Camera pairing config ─────────────────────────────────────────
// Pair 1: index 0 = master, index 1 = slave
// Pair 2: index 2 = master, index 3 = slave
const std::vector<std::string> CAM_SNS = {
    "JCK26010020",  // pair 1 - master
    "JCK26010019",  // pair 1 - slave
    "JCK26010021",  // pair 2 - master
    "JCK26010013"   // pair 2 - slave
};

const std::string SAVE_ROOT = "recordings";

// ── Per-camera state ──────────────────────────────────────────────
struct CameraState
{
    std::atomic<int> frameCount{ 0 };
    std::string savePath;

    CameraState() = default;
    CameraState(CameraState&& other) noexcept
        : frameCount(other.frameCount.load()),
          savePath(std::move(other.savePath)) {}
};

std::vector<CameraState> g_cameras(4);

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

        void* pRGB = imgPtr->ConvertToRGB24(GX_BIT_0_7, GX_RAW2RGB_NEIGHBOUR, true);

        cv::Mat rgbMat(height, width, CV_8UC3, pRGB);
        cv::Mat bgrMat;
        cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
        cv::flip(bgrMat, bgrMat, 0);

        int count = g_cameras[m_index].frameCount.fetch_add(1);

        std::ostringstream ss;
        ss << g_cameras[m_index].savePath
           << "/frame_" << std::setw(6) << std::setfill('0') << count << ".jpg";

        cv::imwrite(ss.str(), bgrMat, { cv::IMWRITE_JPEG_QUALITY, 95 });
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

    // pair_1/camera_0, pair_1/camera_1, pair_2/camera_2, pair_2/camera_3
    for (int pair = 1; pair <= 2; ++pair)
    {
        for (int cam = 0; cam < 2; ++cam)
        {
            int idx = (pair - 1) * 2 + cam;
            std::string path = expPath + "/pair_" + std::to_string(pair)
                             + "/camera_" + std::to_string(idx);
            fs::create_directories(path);
            g_cameras[idx].savePath = path;
        }
    }

    return expPath;
}

// ── Trigger configuration ─────────────────────────────────────────
// Master: free-runs and outputs a trigger pulse on Line1 when it exposes.
// Slave : waits for a rising edge on Line0 before each capture.
// NOTE  : verify Line1 (output) and Line0 (input) match your cable wiring.

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

        // Enumerate cameras first — required before OpenDeviceBySN
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

            // Full resolution
            featureControls[i]->GetIntFeature("Width")->SetValue(2600);
            featureControls[i]->GetIntFeature("Height")->SetValue(2160);
            featureControls[i]->GetIntFeature("OffsetX")->SetValue(0);
            featureControls[i]->GetIntFeature("OffsetY")->SetValue(0);

            // Maximum throughput, fixed frame rate for all cameras
            featureControls[i]->GetIntFeature("DeviceLinkThroughputLimit")->SetValue(400000000);
            featureControls[i]->GetEnumFeature("AcquisitionFrameRateMode")->SetValue("On");
            featureControls[i]->GetFloatFeature("AcquisitionFrameRate")->SetValue(70.0);

            featureControls[i]->GetEnumFeature("AcquisitionMode")->SetValue("Continuous");

            // TODO: revert to master/slave once GPIO trigger cable is connected
            configureMaster(featureControls[i]);

            streams[i] = cameras[i]->OpenStream(0);

            callbacks[i] = std::make_shared<CCaptureHandler>(i);
            streams[i]->RegisterCaptureCallback(callbacks[i].get(), nullptr);
            streams[i]->StartGrab();
            featureControls[i]->GetCommandFeature("AcquisitionStart")->Execute();

            std::cout << "Camera " << i << " (" << CAM_SNS[i] << ") started ["
                      << (i % 2 == 0 ? "master" : "slave") << "]" << std::endl;
        }

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

        // Shutdown
        std::cout << "\nShutting down..." << std::endl;
        for (int i = 0; i < 4; ++i)
        {
            featureControls[i]->GetCommandFeature("AcquisitionStop")->Execute();
            streams[i]->StopGrab();
            streams[i]->UnregisterCaptureCallback();
            streams[i]->Close();
            cameras[i]->Close();
        }

        cv::destroyAllWindows();
        IGXFactory::GetInstance().Uninit();

        std::cout << "\nFrames saved:" << std::endl;
        for (int i = 0; i < 4; ++i)
        {
            std::cout << "  Camera " << i << " (" << CAM_SNS[i] << "): "
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
