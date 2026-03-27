#include "GalaxyIncludes.h"
#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <mutex>

// ── Shared frame storage ──────────────────────────────────────────
struct CameraFrame
{
    cv::Mat frame;
    std::mutex mtx;
    bool hasNewFrame = false;

    CameraFrame() = default;
    CameraFrame(CameraFrame&& other) noexcept
        : frame(std::move(other.frame)), hasNewFrame(other.hasNewFrame) {}
};

std::vector<CameraFrame> g_frames;

// ── Callback class ────────────────────────────────────────────────
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
        size_t bufferSize = width * height * 3;

        void* pRGBBuffer = imgPtr->ConvertToRGB24(GX_BIT_0_7, GX_RAW2RGB_NEIGHBOUR, true);

        cv::Mat rgbMat(height, width, CV_8UC3, pRGBBuffer);
        cv::Mat bgrMat;
        cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
        cv::flip(bgrMat, bgrMat, 0);

        {
            std::lock_guard<std::mutex> lock(g_frames[m_index].mtx);
            g_frames[m_index].frame = bgrMat.clone();
            g_frames[m_index].hasNewFrame = true;
        }
    }
};

// ── Main ──────────────────────────────────────────────────────────
int main()
{
    try
    {
        // Step 1: Initialize SDK
        IGXFactory::GetInstance().Init();
        std::cout << "SDK Initialized." << std::endl;

        // Step 2: Enumerate cameras
        GxIAPICPP::gxdeviceinfo_vector vecDeviceInfo;
        IGXFactory::GetInstance().UpdateDeviceList(1000, vecDeviceInfo);

        int camCount = (int)vecDeviceInfo.size();

        if (camCount == 0)
        {
            std::cout << "No cameras found." << std::endl;
            IGXFactory::GetInstance().Uninit();
            return -1;
        }

        std::cout << "Found " << camCount << " camera(s)." << std::endl;

        // Step 3: Resize frame storage
        g_frames.resize(camCount);

        // Step 4: Open cameras, streams, start acquisition
        std::vector<CGXDevicePointer> cameras(camCount);
        std::vector<CGXStreamPointer> streams(camCount);
        std::vector<CGXFeatureControlPointer> featureControls(camCount);
        std::vector<std::shared_ptr<CCaptureHandler>> callbacks(camCount);

        // Phase 1: Open and configure all cameras first
        for (int i = 0; i < camCount; ++i)
        {
            cameras[i] = IGXFactory::GetInstance().OpenDeviceBySN(
                vecDeviceInfo[i].GetSN(), GX_ACCESS_EXCLUSIVE
            );

            featureControls[i] = cameras[i]->GetRemoteFeatureControl();

            featureControls[i]->GetIntFeature("Width")->SetValue(2600);
            featureControls[i]->GetIntFeature("Height")->SetValue(2160);
            featureControls[i]->GetIntFeature("OffsetX")->SetValue(0);
            featureControls[i]->GetIntFeature("OffsetY")->SetValue(0);

            featureControls[i]->GetEnumFeature("AcquisitionMode")->SetValue("Continuous");

            streams[i] = cameras[i]->OpenStream(0);
            callbacks[i] = std::make_shared<CCaptureHandler>(i);
            streams[i]->RegisterCaptureCallback(callbacks[i].get(), nullptr);

            std::cout << "Camera " << i << " configured." << std::endl;
        }

        // Phase 2: Start all cameras streaming together
        for (int i = 0; i < camCount; ++i)
        {
            streams[i]->StartGrab();
            featureControls[i]->GetCommandFeature("AcquisitionStart")->Execute();
            std::cout << "Camera " << i << " started." << std::endl;
        }

        // Step 5: Display loop
        std::cout << "Press Q to quit." << std::endl;

        // Pre-create windows so waitKey works immediately
        for (int i = 0; i < camCount; ++i)
        {
            std::string winName = "Camera " + std::to_string(i)
                                + " [" + std::string(vecDeviceInfo[i].GetSN()) + "]";
            cv::namedWindow(winName, cv::WINDOW_NORMAL);
            cv::resizeWindow(winName, 800, 667);
        }

        while (true)
        {
            for (int i = 0; i < camCount; ++i)
            {
                cv::Mat display;
                {
                    std::lock_guard<std::mutex> lock(g_frames[i].mtx);
                    if (g_frames[i].hasNewFrame)
                    {
                        display = g_frames[i].frame.clone();
                        g_frames[i].hasNewFrame = false;
                    }
                }

                if (!display.empty())
                {
                    std::string winName = "Camera " + std::to_string(i)
                                       + " [" + std::string(vecDeviceInfo[i].GetSN()) + "]";
                    cv::imshow(winName, display);
                }
            }

            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 'Q' || key == 27)
                break;
        }

        // Step 6: Clean shutdown
        std::cout << "Shutting down..." << std::endl;
        for (int i = 0; i < camCount; ++i)
        {
            featureControls[i]->GetCommandFeature("AcquisitionStop")->Execute();
            streams[i]->StopGrab();
            streams[i]->UnregisterCaptureCallback();
            streams[i]->Close();
            cameras[i]->Close();
        }

        cv::destroyAllWindows();
        IGXFactory::GetInstance().Uninit();
        std::cout << "Done." << std::endl;
    }
    catch (CGalaxyException& e)
    {
        std::cout << "Camera error: " << e.what() << std::endl;
        return -1;
    }
    catch (std::exception& e)
    {
        std::cout << "General error: " << e.what() << std::endl;
        return -1;
    }

    return 0;
}
