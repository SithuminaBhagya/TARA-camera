#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>
#include <iomanip>
#include <sstream>

namespace fs = std::filesystem;

// ── Config ────────────────────────────────────────────────────────
const std::string SAVE_ROOT  = "recordings";
const int         DISPLAY_W  = 800;   // width of each camera cell
const int         DISPLAY_H  = 667;   // height of each camera cell
const int         FPS        = 70;    // playback FPS — match capture rate

// ── Collect sorted frame paths for one camera folder ──────────────
std::vector<std::string> getFramePaths(const std::string& camFolder)
{
    std::vector<std::string> paths;
    for (const auto& entry : fs::directory_iterator(camFolder))
    {
        if (entry.path().extension() == ".jpg")
            paths.push_back(entry.path().string());
    }
    std::sort(paths.begin(), paths.end());
    return paths;
}

// ── List available experiments ────────────────────────────────────
std::vector<std::string> listExperiments()
{
    std::vector<std::string> experiments;
    if (!fs::exists(SAVE_ROOT))
        return experiments;
    for (const auto& entry : fs::directory_iterator(SAVE_ROOT))
    {
        if (entry.is_directory())
            experiments.push_back(entry.path().string());
    }
    std::sort(experiments.begin(), experiments.end());
    return experiments;
}

// ── Build 2x2 grid from 4 frames ─────────────────────────────────
cv::Mat buildGrid(const std::vector<cv::Mat>& frames, int frameIdx, int totalFrames)
{
    std::vector<cv::Mat> cells(4);

    for (int i = 0; i < 4; ++i)
    {
        cv::Mat cell;
        if (frames[i].empty())
        {
            cell = cv::Mat(DISPLAY_H, DISPLAY_W, CV_8UC3, cv::Scalar(30, 30, 30));
            cv::putText(cell, "No frame", { DISPLAY_W / 2 - 50, DISPLAY_H / 2 },
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, { 100, 100, 100 }, 2);
        }
        else
        {
            cv::resize(frames[i], cell, { DISPLAY_W, DISPLAY_H });
        }

        // Label overlay
        int pair = (i / 2) + 1;
        std::string label = "Pair " + std::to_string(pair)
                          + "  Cam " + std::to_string(i);
        cv::putText(cell, label, { 10, 30 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, { 0, 255, 0 }, 2);

        // Frame counter
        std::ostringstream ss;
        ss << frameIdx << " / " << totalFrames;
        cv::putText(cell, ss.str(), { 10, DISPLAY_H - 10 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, { 200, 200, 200 }, 1);

        cells[i] = cell;
    }

    // Stack into 2x2 grid
    cv::Mat top, bottom, grid;
    cv::hconcat(cells[0], cells[1], top);
    cv::hconcat(cells[2], cells[3], bottom);
    cv::vconcat(top, bottom, grid);
    return grid;
}

// ── Main ──────────────────────────────────────────────────────────
int main()
{
    // List experiments
    auto experiments = listExperiments();
    if (experiments.empty())
    {
        std::cout << "No experiments found in: " << SAVE_ROOT << std::endl;
        return -1;
    }

    std::cout << "Available experiments:" << std::endl;
    for (int i = 0; i < (int)experiments.size(); ++i)
        std::cout << "  [" << i << "] " << experiments[i] << std::endl;

    std::cout << "Select experiment number: ";
    int choice = 0;
    std::cin >> choice;

    if (choice < 0 || choice >= (int)experiments.size())
    {
        std::cout << "Invalid choice." << std::endl;
        return -1;
    }

    std::string expPath = experiments[choice];

    // Build camera folder paths
    std::vector<std::string> camFolders = {
        expPath + "/pair_1/camera_0",
        expPath + "/pair_1/camera_1",
        expPath + "/pair_2/camera_2",
        expPath + "/pair_2/camera_3"
    };

    // Load frame paths
    std::vector<std::vector<std::string>> allFramePaths(4);
    int maxFrames = 0;
    for (int i = 0; i < 4; ++i)
    {
        if (fs::exists(camFolders[i]))
            allFramePaths[i] = getFramePaths(camFolders[i]);
        maxFrames = std::max(maxFrames, (int)allFramePaths[i].size());
        std::cout << "Camera " << i << ": " << allFramePaths[i].size()
                  << " frames" << std::endl;
    }

    if (maxFrames == 0)
    {
        std::cout << "No frames found." << std::endl;
        return -1;
    }

    std::cout << "\nControls: SPACE = pause/resume | Q = quit\n" << std::endl;

    cv::namedWindow("Playback", cv::WINDOW_NORMAL);
    cv::resizeWindow("Playback", DISPLAY_W * 2, DISPLAY_H * 2);

    int  frameIdx  = 0;
    bool paused    = false;
    int  delay     = 1000 / FPS;

    while (true)
    {
        if (!paused)
        {
            std::vector<cv::Mat> frames(4);
            for (int i = 0; i < 4; ++i)
            {
                if (frameIdx < (int)allFramePaths[i].size())
                    frames[i] = cv::imread(allFramePaths[i][frameIdx]);
            }

            cv::Mat grid = buildGrid(frames, frameIdx, maxFrames);
            cv::imshow("Playback", grid);

            ++frameIdx;
            if (frameIdx >= maxFrames)
                frameIdx = 0;  // loop
        }

        int key = cv::waitKey(paused ? 30 : delay) & 0xFF;
        if (key == 'q' || key == 'Q' || key == 27)
            break;
        if (key == ' ')
            paused = !paused;

        if (cv::getWindowProperty("Playback", cv::WND_PROP_VISIBLE) < 1)
            break;
    }

    cv::destroyAllWindows();
    return 0;
}
