#include "GalaxyIncludes.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <mutex>
#include <atomic>
#include <chrono>
#include <sstream>
#include <iomanip>

// ── Camera SNs ────────────────────────────────────────────────────
const std::vector<std::string> CAM_SNS = {
    "JCK26010020",  // pair 1 - camera 0
    "JCK26010019",  // pair 1 - camera 1
    "JCK26010021",  // pair 2 - camera 2
    "JCK26010013"   // pair 2 - camera 3
};

// ── Display config ────────────────────────────────────────────────
const int DISPLAY_W = 800;
const int DISPLAY_H = 667;

// ── Per-camera state ──────────────────────────────────────────────
struct CameraState
{
    cv::Mat  frame;
    std::mutex mtx;
    bool hasNewFrame = false;

    std::atomic<int>    frameCount{ 0 };
    std::atomic<int>    fpsCount{ 0 };    // frames since last FPS update
    std::atomic<double> fps{ 0.0 };
    std::atomic<double> throughputMBs{ 0.0 };

    CameraState() = default;
    CameraState(CameraState&& other) noexcept
        : frame(std::move(other.frame)),
          hasNewFrame(other.hasNewFrame),
          frameCount(other.frameCount.load()),
          fpsCount(other.fpsCount.load()),
          fps(other.fps.load()),
          throughputMBs(other.throughputMBs.load()) {}
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

        auto& cam = g_cameras[m_index];
        cam.frameCount.fetch_add(1);
        cam.fpsCount.fetch_add(1);

        std::lock_guard<std::mutex> lock(cam.mtx);
        cam.frame      = bgrMat.clone();
        cam.hasNewFrame = true;
    }
};

// ── Build 2x2 grid ────────────────────────────────────────────────
cv::Mat buildGrid(const std::vector<cv::Mat>& frames)
{
    std::vector<cv::Mat> cells(4);

    for (int i = 0; i < 4; ++i)
    {
        cv::Mat cell;
        if (frames[i].empty())
            cell = cv::Mat(DISPLAY_H, DISPLAY_W, CV_8UC3, cv::Scalar(30, 30, 30));
        else
            cv::resize(frames[i], cell, { DISPLAY_W, DISPLAY_H });

        // Pair and camera label
        int pair = (i / 2) + 1;
        std::string label = "Pair " + std::to_string(pair)
                          + "  Cam " + std::to_string(i)
                          + "  [" + CAM_SNS[i] + "]";
        cv::putText(cell, label, { 10, 30 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 0, 255, 0 }, 2);

        // FPS
        std::ostringstream fpsSS;
        fpsSS << std::fixed << std::setprecision(1)
              << "FPS: " << g_cameras[i].fps.load();
        cv::putText(cell, fpsSS.str(), { 10, 65 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 0, 255, 255 }, 2);

        // Throughput
        std::ostringstream tpSS;
        tpSS << std::fixed << std::setprecision(1)
             << "Throughput: " << g_cameras[i].throughputMBs.load() << " MB/s";
        cv::putText(cell, tpSS.str(), { 10, 100 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 0, 200, 255 }, 2);

        // Total frames
        std::string frameSS = "Frames: " + std::to_string(g_cameras[i].frameCount.load());
        cv::putText(cell, frameSS, { 10, DISPLAY_H - 10 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, { 200, 200, 200 }, 1);

        cells[i] = cell;
    }

    cv::Mat top, bottom, grid;
    cv::hconcat(cells[0], cells[1], top);
    cv::hconcat(cells[2], cells[3], bottom);
    cv::vconcat(top, bottom, grid);
    return grid;
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

        std::vector<CGXDevicePointer>                 cameras(4);
        std::vector<CGXStreamPointer>                 streams(4);
        std::vector<CGXFeatureControlPointer>         featureControls(4);
        std::vector<std::shared_ptr<CCaptureHandler>> callbacks(4);

        // Phase 1: Open and configure all cameras first
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
            featureControls[i]->GetEnumFeature("AcquisitionFrameRateMode")->SetValue("On");
            featureControls[i]->GetFloatFeature("AcquisitionFrameRate")->SetValue(70.0);

            featureControls[i]->GetEnumFeature("AcquisitionMode")->SetValue("Continuous");
            featureControls[i]->GetEnumFeature("TriggerMode")->SetValue("Off");

            streams[i] = cameras[i]->OpenStream(0);
            callbacks[i] = std::make_shared<CCaptureHandler>(i);
            streams[i]->RegisterCaptureCallback(callbacks[i].get(), nullptr);

            std::cout << "Camera " << i << " (" << CAM_SNS[i] << ") configured." << std::endl;
        }

        // Phase 2: Start all cameras streaming together
        for (int i = 0; i < 4; ++i)
        {
            streams[i]->StartGrab();
            featureControls[i]->GetCommandFeature("AcquisitionStart")->Execute();
            std::cout << "Camera " << i << " (" << CAM_SNS[i] << ") started." << std::endl;
        }

        std::cout << "Press Q or close window to quit." << std::endl;

        cv::namedWindow("Live Feed", cv::WINDOW_NORMAL);
        cv::resizeWindow("Live Feed", DISPLAY_W * 2, DISPLAY_H * 2);

        // FPS/throughput update timer
        auto lastFPSUpdate = std::chrono::steady_clock::now();
        const double frameBytes = 2600.0 * 2160.0 * 3.0; // RGB24

        while (true)
        {
            // Update FPS and throughput every second
            auto now = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - lastFPSUpdate).count();
            if (elapsed >= 1.0)
            {
                for (int i = 0; i < 4; ++i)
                {
                    int count = g_cameras[i].fpsCount.exchange(0);
                    double fps = count / elapsed;
                    g_cameras[i].fps.store(fps);
                    g_cameras[i].throughputMBs.store((fps * frameBytes) / (1024.0 * 1024.0));
                }
                lastFPSUpdate = now;
            }

            // Grab latest frames
            std::vector<cv::Mat> frames(4);
            for (int i = 0; i < 4; ++i)
            {
                std::lock_guard<std::mutex> lock(g_cameras[i].mtx);
                if (g_cameras[i].hasNewFrame)
                {
                    frames[i] = g_cameras[i].frame.clone();
                    g_cameras[i].hasNewFrame = false;
                }
            }

            cv::Mat grid = buildGrid(frames);
            cv::imshow("Live Feed", grid);

            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 'Q' || key == 27)
                break;
            if (cv::getWindowProperty("Live Feed", cv::WND_PROP_VISIBLE) < 1)
                break;
        }

        // Shutdown
        std::cout << "Shutting down..." << std::endl;
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
        std::cout << "Done." << std::endl;
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
