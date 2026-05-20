#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <filesystem>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <thread>
#include <cmath>
#include <numeric>

namespace fs = std::filesystem;

// ── Config ────────────────────────────────────────────────────────
const std::string SAVE_ROOT = "recordings";
const int         DISPLAY_W = 800;
const int         DISPLAY_H = 667;
const int         FPS       = 70;
const int         IMG_W     = 2600;
const int         IMG_H     = 2160;
const size_t      FRAME_BYTES = (size_t)IMG_W * IMG_H;

const std::vector<std::string> CAM_LABELS = {
    "Cam 1  P1  master 10021",
    "Cam 2  P1  slave  10020",
    "Cam 3  P2  slave  10019",
    "Cam 4  P2  slave  10013"
};

// ── Read frame stride from metadata.txt (falls back to FRAME_BYTES for old recordings) ──
size_t getFrameStride(const std::string& camFolder)
{
    std::ifstream meta(camFolder + "/metadata.txt");
    if (meta.is_open())
    {
        std::string line;
        while (std::getline(meta, line))
        {
            if (line.substr(0, 13) == "frame_stride=")
                return std::stoull(line.substr(13));
        }
    }
    return FRAME_BYTES; // old experiments had no padding
}

// ── Read frame count from metadata.txt ───────────────────────────
int getFrameCount(const std::string& camFolder)
{
    std::ifstream meta(camFolder + "/metadata.txt");
    if (meta.is_open())
    {
        std::string line;
        while (std::getline(meta, line))
        {
            if (line.substr(0, 7) == "frames=")
                return std::stoi(line.substr(7));
        }
    }
    // Fallback: count rows in timestamps.csv (minus header)
    std::ifstream ts(camFolder + "/timestamps.csv");
    if (ts.is_open())
    {
        int count = -1;
        std::string line;
        while (std::getline(ts, line)) ++count;
        return std::max(0, count);
    }
    return 0;
}

// ── Load a single frame from frames.bin by index ─────────────────
cv::Mat loadFrame(const std::string& camFolder, int frameIdx, size_t stride)
{
    std::ifstream f(camFolder + "/frames.bin", std::ios::binary);
    if (!f.is_open()) return {};

    f.seekg((std::streamoff)frameIdx * (std::streamoff)stride);
    if (!f) return {};

    cv::Mat gray(IMG_H, IMG_W, CV_8UC1);
    f.read(reinterpret_cast<char*>(gray.data), FRAME_BYTES);
    if (!f) return {};

    return gray;
}

// ── Load timestamps from CSV ──────────────────────────────────────
// CSV format: saved_index,frame_index,timestamp_ticks
// Assumes 1 tick = 1 ns (1 GHz camera clock).
std::vector<uint64_t> loadTimestamps(const std::string& camFolder)
{
    std::vector<uint64_t> ts;
    std::ifstream f(camFolder + "/timestamps.csv");
    if (!f.is_open()) return ts;
    std::string line;
    std::getline(f, line); // skip header
    while (std::getline(f, line))
    {
        auto c1 = line.find(',');
        if (c1 == std::string::npos) continue;
        auto c2 = line.find(',', c1 + 1);
        if (c2 == std::string::npos) continue;
        ts.push_back(std::stoull(line.substr(c2 + 1)));
    }
    return ts;
}

// ── Print sync statistics ─────────────────────────────────────────
void printSyncStats(const std::vector<std::vector<uint64_t>>& allTs,
                    const std::vector<int>& frameCounts)
{
    std::cout << "\n=== Sync Statistics ===" << std::endl;
    std::cout << std::left
              << std::setw(26) << "Camera"
              << std::setw(9)  << "Frames"
              << std::setw(14) << "Mean IFT"
              << "Jitter (1-sigma)" << std::endl;
    std::cout << std::string(65, '-') << std::endl;

    int baseCount = -1;
    bool countMatch = true;

    for (int i = 0; i < 4; ++i)
    {
        int n = frameCounts[i];
        if (baseCount == -1) baseCount = n;
        else if (n != baseCount) countMatch = false;

        std::cout << std::left << std::setw(30) << CAM_LABELS[i]
                  << std::setw(9) << n;

        const auto& ts = allTs[i];
        if (ts.size() > 1)
        {
            std::vector<double> diffs;
            diffs.reserve(ts.size() - 1);
            for (size_t j = 1; j < ts.size(); ++j)
                diffs.push_back((double)(ts[j] - ts[j - 1]) / 1e6); // ns → ms

            double mean = std::accumulate(diffs.begin(), diffs.end(), 0.0) / diffs.size();
            double var  = 0.0;
            for (double d : diffs) var += (d - mean) * (d - mean);
            double sigma = std::sqrt(var / diffs.size());

            std::cout << std::fixed << std::setprecision(3)
                      << std::setw(9) << mean << " ms   "
                      << std::setw(7) << sigma << " ms";
        }
        else
        {
            std::cout << "  (no timestamp data)";
        }
        std::cout << std::endl;
    }

    std::cout << std::string(65, '-') << std::endl;
    if (countMatch)
        std::cout << "Frame count match: YES  (all cameras captured same number of frames)" << std::endl;
    else
        std::cout << "Frame count match: NO   -- missed triggers detected, sync may be broken!" << std::endl;

    std::cout << "======================\n" << std::endl;
}

// ── List available experiments ────────────────────────────────────
std::vector<std::string> listExperiments()
{
    std::vector<std::string> experiments;
    if (!fs::exists(SAVE_ROOT)) return experiments;
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
            cv::putText(cell, "No frame", { DISPLAY_W / 2 - 60, DISPLAY_H / 2 },
                        cv::FONT_HERSHEY_SIMPLEX, 1.0, { 100, 100, 100 }, 2);
        }
        else
        {
            cv::Mat bgr;
            cv::cvtColor(frames[i], bgr, cv::COLOR_GRAY2BGR);
            cv::resize(bgr, cell, { DISPLAY_W, DISPLAY_H });
        }

        cv::putText(cell, CAM_LABELS[i], { 10, 30 },
                    cv::FONT_HERSHEY_SIMPLEX, 0.7, { 0, 255, 0 }, 2);

        std::ostringstream ss;
        ss << "frame " << frameIdx << " / " << totalFrames;
        cv::putText(cell, ss.str(), { 10, DISPLAY_H - 10 },
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

    std::vector<std::string> camFolders = {
        expPath + "/camera_1",
        expPath + "/camera_2",
        expPath + "/camera_3",
        expPath + "/camera_4"
    };

    std::vector<std::vector<uint64_t>> allTimestamps(4);
    std::vector<int>                   frameCounts(4, 0);
    std::vector<size_t>                frameStrides(4, FRAME_BYTES);
    int maxFrames = 0;

    for (int i = 0; i < 4; ++i)
    {
        frameCounts[i]   = getFrameCount(camFolders[i]);
        frameStrides[i]  = getFrameStride(camFolders[i]);
        allTimestamps[i] = loadTimestamps(camFolders[i]);
        maxFrames        = std::max(maxFrames, frameCounts[i]);
        std::cout << "Camera " << (i + 1) << ": " << frameCounts[i] << " frames  "
                  << allTimestamps[i].size() << " timestamps" << std::endl;
    }

    if (maxFrames == 0)
    {
        std::cout << "No frames found." << std::endl;
        return -1;
    }

    printSyncStats(allTimestamps, frameCounts);

    std::cout << "Controls: SPACE = pause/resume | Q = quit\n" << std::endl;

    cv::namedWindow("Playback", cv::WINDOW_NORMAL);
    cv::resizeWindow("Playback", DISPLAY_W * 2, DISPLAY_H * 2);

    int  frameIdx      = 0;
    bool paused        = false;
    auto frameDuration = std::chrono::microseconds(1000000 / FPS);

    while (true)
    {
        auto frameStart = std::chrono::steady_clock::now();

        if (!paused)
        {
            std::vector<cv::Mat> frames(4);
            for (int i = 0; i < 4; ++i)
            {
                if (frameIdx < frameCounts[i])
                    frames[i] = loadFrame(camFolders[i], frameIdx, frameStrides[i]);
            }

            cv::Mat grid = buildGrid(frames, frameIdx, maxFrames);
            cv::imshow("Playback", grid);

            ++frameIdx;
            if (frameIdx >= maxFrames)
                frameIdx = 0;
        }

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q' || key == 'Q' || key == 27)
            break;
        if (key == ' ')
            paused = !paused;

        if (cv::getWindowProperty("Playback", cv::WND_PROP_VISIBLE) < 1)
            break;

        auto elapsed   = std::chrono::steady_clock::now() - frameStart;
        auto sleepTime = frameDuration - elapsed;
        if (sleepTime > std::chrono::microseconds(0))
            std::this_thread::sleep_for(sleepTime);
    }

    cv::destroyAllWindows();
    return 0;
}
