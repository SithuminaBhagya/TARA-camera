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

// index 0 = master (JCK26010021 / SN 10021), indices 1-3 = slaves
const std::vector<std::string> CAM_SNS = {
    "JCK26010021",  // 0 — master
    "JCK26010020",  // 1 — slave
    "JCK26010019",  // 2 — slave
    "JCK26010013"   // 3 — slave
};

const int DISPLAY_W = 800;
const int DISPLAY_H = 667;

// ── Per-camera state ──────────────────────────────────────────────
struct CameraState
{
    cv::Mat  frame;
    std::mutex mtx;
    bool hasNewFrame = false;

    std::atomic<int>      frameCount{ 0 };
    std::atomic<int>      fpsCount{ 0 };
    std::atomic<double>   fps{ 0.0 };
    std::atomic<int64_t>  lastFrameMs{ -1 };  // wall-clock ms when last frame arrived

    CameraState() = default;
    CameraState(CameraState&& other) noexcept
        : frame(std::move(other.frame)),
          hasNewFrame(other.hasNewFrame),
          frameCount(other.frameCount.load()),
          fpsCount(other.fpsCount.load()),
          fps(other.fps.load()),
          lastFrameMs(other.lastFrameMs.load()) {}
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

        void* pRaw8 = imgPtr->ConvertToRaw8(GX_BIT_0_7);
        cv::Mat grayMat(height, width, CV_8UC1, pRaw8);
        cv::Mat bgrMat;
        cv::cvtColor(grayMat, bgrMat, cv::COLOR_GRAY2BGR);

        auto& cam = g_cameras[m_index];
        cam.frameCount.fetch_add(1);
        cam.fpsCount.fetch_add(1);
        cam.lastFrameMs.store(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now().time_since_epoch()).count());

        std::lock_guard<std::mutex> lock(cam.mtx);
        cam.frame       = bgrMat.clone();
        cam.hasNewFrame = true;
    }
};

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
    fc->GetEnumFeature("TriggerMode")->SetValue("On");
    fc->GetEnumFeature("TriggerSource")->SetValue("Line0");
    fc->GetEnumFeature("TriggerActivation")->SetValue("RisingEdge");
}

// ── Sync status helpers ───────────────────────────────────────────
// delta = slave frameCount - master frameCount
// In hardware sync this stays near 0. If circuit is broken, slaves
// get no trigger and delta grows negatively without bound.
struct SyncStatus { cv::Scalar border; std::string text; cv::Scalar textColor; };

SyncStatus evalSync(int camIndex)
{
    if (camIndex == 0)
        return { { 0, 200, 0 }, "MASTER", { 0, 255, 0 } };

    double slaveFPS = g_cameras[camIndex].fps.load();
    int delta = g_cameras[camIndex].frameCount.load()
              - g_cameras[0].frameCount.load();

    if (slaveFPS < 1.0)
        return { { 0, 0, 220 }, "NO TRIGGER  check wiring", { 0, 80, 255 } };

    int absDelta = std::abs(delta);
    if (absDelta <= 3)
        return { { 0, 200, 0 }, "SYNC OK", { 0, 255, 0 } };
    if (absDelta <= 15)
        return { { 0, 165, 255 }, "MINOR DRIFT  d=" + std::to_string(delta), { 0, 165, 255 } };

    return { { 0, 0, 220 }, "SYNC FAIL  d=" + std::to_string(delta), { 0, 80, 255 } };
}

// ── Build 2x2 grid ────────────────────────────────────────────────
cv::Mat buildGrid(const std::vector<cv::Mat>& frames)
{
    std::vector<cv::Mat> cells(4);
    int masterCount = g_cameras[0].frameCount.load();

    int64_t nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();

    for (int i = 0; i < 4; ++i)
    {
        cv::Mat cell;
        if (frames[i].empty())
            cell = cv::Mat(DISPLAY_H, DISPLAY_W, CV_8UC3, cv::Scalar(30, 30, 30));
        else
            cv::resize(frames[i], cell, { DISPLAY_W, DISPLAY_H });

        auto sync = evalSync(i);

        // ── Green flash overlay when a frame just arrived ─────────
        // All 4 cells flash green at the same instant when in sync —
        // visually obvious even at 70fps.
        int64_t lastMs = g_cameras[i].lastFrameMs.load();
        int64_t age    = (lastMs < 0) ? 999999 : (nowMs - lastMs);
        const int64_t FLASH_MS = 120;
        if (age < FLASH_MS)
        {
            double alpha = 0.45 * (1.0 - (double)age / FLASH_MS);
            cv::Mat green(cell.size(), cell.type(), cv::Scalar(0, 220, 0));
            cv::addWeighted(cell, 1.0 - alpha, green, alpha, 0, cell);
        }

        // Colored border shows sync health at a glance
        cv::rectangle(cell, { 0, 0 }, { DISPLAY_W - 1, DISPLAY_H - 1 }, sync.border, 8);

        // Camera label
        std::string role  = (i == 0) ? "MASTER" : "SLAVE";
        std::string label = "Cam " + std::to_string(i) + "  " + role
                          + "  [" + CAM_SNS[i].substr(8) + "]";
        cv::putText(cell, label, { 14, 35 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, { 0, 255, 0 }, 2);

        // FPS — red if near zero (no trigger arriving)
        double fps = g_cameras[i].fps.load();
        cv::Scalar fpsColor = (fps < 1.0) ? cv::Scalar(0, 80, 255) : cv::Scalar(0, 255, 255);
        std::ostringstream fpsSS;
        fpsSS << std::fixed << std::setprecision(1) << "FPS: " << fps;
        cv::putText(cell, fpsSS.str(), { 14, 72 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, fpsColor, 2);

        // Sync status text
        cv::putText(cell, sync.text, { 14, 109 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.75, sync.textColor, 2);

        // Delta frame count (slaves only)
        if (i > 0)
        {
            int delta = g_cameras[i].frameCount.load() - masterCount;
            std::string dStr = "Delta frames: " + (delta >= 0 ? std::string("+") : "") + std::to_string(delta);
            cv::Scalar dColor = (std::abs(delta) <= 3) ? cv::Scalar(200, 200, 200)
                                                       : cv::Scalar(0, 80, 255);
            cv::putText(cell, dStr, { 14, 146 },
                        cv::FONT_HERSHEY_SIMPLEX, 0.65, dColor, 2);
        }

        // Time since last frame — if cameras are in sync these should
        // all read nearly the same value at the same moment
        if (lastMs >= 0)
        {
            std::string ageStr = "Last frame: " + std::to_string(age) + " ms ago";
            cv::Scalar ageColor = (age < 50) ? cv::Scalar(0, 255, 0) : cv::Scalar(0, 165, 255);
            cv::putText(cell, ageStr, { 14, 183 },
                        cv::FONT_HERSHEY_SIMPLEX, 0.65, ageColor, 2);
        }

        // "WAITING FOR TRIGGER" — proves slave is in trigger mode
        // and blocking correctly before first frame arrives
        if (g_cameras[i].frameCount.load() == 0 && i > 0)
        {
            cv::Mat dark = cell.clone();
            cv::addWeighted(cell, 0.4, dark, 0.0, 0, cell);
            cv::putText(cell, "WAITING FOR", { DISPLAY_W / 2 - 160, DISPLAY_H / 2 - 20 },
                        cv::FONT_HERSHEY_SIMPLEX, 1.4, { 0, 255, 255 }, 3);
            cv::putText(cell, "TRIGGER...", { DISPLAY_W / 2 - 135, DISPLAY_H / 2 + 50 },
                        cv::FONT_HERSHEY_SIMPLEX, 1.4, { 0, 255, 255 }, 3);
        }

        // Total frames (bottom)
        std::string frameSS = "Total: " + std::to_string(g_cameras[i].frameCount.load());
        cv::putText(cell, frameSS, { 14, DISPLAY_H - 12 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, { 180, 180, 180 }, 1);

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

        // Arm all buffer queues first
        for (int i = 0; i < 4; ++i)
            streams[i]->StartGrab();

        // Start slaves first — they block waiting for the trigger signal
        for (int i = 1; i < 4; ++i)
        {
            featureControls[i]->GetCommandFeature("AcquisitionStart")->Execute();
            std::cout << "Slave " << i << " (" << CAM_SNS[i] << ") armed." << std::endl;
        }

        // Start master last — begins sending trigger pulses
        featureControls[0]->GetCommandFeature("AcquisitionStart")->Execute();
        std::cout << "Master (" << CAM_SNS[0] << ") started.\n" << std::endl;

        std::cout << "== Circuit check ==" << std::endl;
        std::cout << "  Green border  + SYNC OK     -> trigger working, cameras in sync" << std::endl;
        std::cout << "  Blue  border  + NO TRIGGER  -> circuit broken, check wiring" << std::endl;
        std::cout << "  Orange border + MINOR DRIFT -> transient, watch if it grows" << std::endl;
        std::cout << "Press Q or close window to quit.\n" << std::endl;

        cv::namedWindow("Live Feed  |  Sync Test", cv::WINDOW_NORMAL);
        cv::resizeWindow("Live Feed  |  Sync Test", DISPLAY_W * 2, DISPLAY_H * 2);

        auto lastFPSUpdate = std::chrono::steady_clock::now();

        while (true)
        {
            auto now     = std::chrono::steady_clock::now();
            double elapsed = std::chrono::duration<double>(now - lastFPSUpdate).count();
            if (elapsed >= 1.0)
            {
                for (int i = 0; i < 4; ++i)
                {
                    int count = g_cameras[i].fpsCount.exchange(0);
                    g_cameras[i].fps.store(count / elapsed);
                }
                lastFPSUpdate = now;
            }

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
            cv::imshow("Live Feed  |  Sync Test", grid);

            int key = cv::waitKey(1) & 0xFF;
            if (key == 'q' || key == 'Q' || key == 27)
                break;
            if (cv::getWindowProperty("Live Feed  |  Sync Test", cv::WND_PROP_VISIBLE) < 1)
                break;
        }

        // Stop master first, then slaves
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
