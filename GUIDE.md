# Daheng Industrial Camera SDK
## Complete Learning Guide — Updated Edition
### C++ | VS Code | Windows x64 | USB3 Vision | OpenCV Display

---

**What this guide covers:**
- Setting up your development environment from scratch
- Understanding CMake and how to configure a C++ project
- Understanding the Daheng SDK folder structure
- Writing C++ code to find, open, and stream cameras
- Adding OpenCV for live display of multiple camera feeds
- Building, running, and troubleshooting ALL errors encountered

---

## Chapter 1: Understanding the Tools

Before writing a single line of code, you need to understand what each tool does and why it exists. This chapter explains every tool in your setup.

### 1.1 The Compiler (MSVC / cl.exe)

A compiler is a program that reads your human-readable C++ code and converts it into machine code that your computer can actually execute. Without a compiler, your .cpp files are just text files — they do nothing.

You installed Microsoft Visual Studio 2026 Community edition. This comes with the MSVC compiler. The compiler executable is called cl.exe.

Why MSVC specifically? The Daheng SDK library files (.lib) were compiled using MSVC. When you mix compilers (e.g. using GCC to link against an MSVC-compiled library), you often get errors. Using the same compiler family avoids this.

The compiler does not load automatically in a regular terminal. You must use the special developer terminal that sets up all the necessary environment variables. This is called the x64 Native Tools Command Prompt or you can activate it manually with:

```
cmd /k "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
```

NOTE: The number 18 in the path is the internal version number of Visual Studio 2026. Microsoft uses internal version numbers that do not always match the year shown in the product name.

To verify the compiler is working, run:
```
cl
```
You should see: `Microsoft (R) C/C++ Optimizing Compiler Version 19.xx for x64`

### 1.2 CMake

CMake is a build system generator. It does NOT compile your code directly. Instead, it reads your CMakeLists.txt file and generates the instructions that the compiler needs to build your project.

Think of it this way: CMake is the architect who draws the blueprint. The compiler is the builder who follows it.

CMake solves a real problem: different platforms use different build systems. On Windows you might use NMake or Visual Studio. On Linux you use Make. On Mac you might use Xcode. CMake lets you write one CMakeLists.txt and build on all of them.

To check your CMake version:
```
cmake --version
```

The build process with CMake has two steps:
- **Step 1 — Configure step:** CMake reads CMakeLists.txt and generates build files
- **Step 2 — Build step:** The compiler uses those build files to compile your code

### 1.3 The Daheng SDK

SDK stands for Software Development Kit. The Daheng GxIAPICPPEx SDK is a collection of pre-written code that handles all the low-level USB3 Vision camera communication for you. Without it, you would have to write thousands of lines of code just to send a command to a camera.

The SDK has three parts you need to know about:

- **Header files (.h)** — these describe what functions and classes exist in the SDK. When the compiler sees `#include "GalaxyIncludes.h"` it reads these files to understand what IGXFactory, CGXDevicePointer etc. mean.
  Location: `C:/Program Files/Daheng Imaging/GalaxySDK/Development/C++ SDK/inc/`

- **Library file (.lib)** — this is pre-compiled SDK code. Your program links against it at build time. It contains the actual implementation of all the functions described in the headers.
  Location: `C:/Program Files/Daheng Imaging/GalaxySDK/Development/C++ SDK/lib/x64/GxIAPICPPEx.lib`

- **DLL file (.dll)** — this is needed at runtime (when your .exe actually runs). The .lib file tells the compiler where to find functions, the .dll contains those functions when the program runs.
  Location: `C:/Program Files/Daheng Imaging/GalaxySDK/APIDLL/`

The relationship between these three:
```
Your .cpp code  -->  needs .h   to COMPILE
Your .exe file  -->  needs .lib to LINK
Your .exe file  -->  needs .dll to RUN
```

### 1.4 OpenCV

OpenCV (Open Source Computer Vision Library) is a widely-used library for image processing and display. The Daheng SDK can give you raw image data from the camera, but it cannot show it on screen. OpenCV handles the display window, frame rendering, and keyboard input for quitting.

Just like the Daheng SDK, OpenCV has include files, lib files, and DLL files that you need to point CMake to.

**Installation:** Download from opencv.org/releases and extract to `C:\opencv`. No installer needed — just extract and point CMake to it.

**OpenCV paths on this machine:**
```
Headers : C:/opencv/build/include
Library : C:/opencv/build/x64/vc16/lib/opencv_world4120.lib
DLL     : C:/opencv/build/x64/vc16/bin/opencv_world4120.dll
```

The DLL must be copied next to your .exe after each clean build:
```
copy "C:\opencv\build\x64\vc16\bin\opencv_world4120.dll" build\
```

### 1.5 VS Code

VS Code (Visual Studio Code) is just a text editor with extra features. It is NOT a compiler and it is NOT Visual Studio (despite the similar name). It is where you write your code. The actual compiling happens through CMake and cl.exe in the terminal.

Important distinction: Visual Studio (the full IDE) has its own built-in build system. VS Code does not — it relies on external tools like CMake and the terminal.

---

## Chapter 2: The SDK Folder Structure

When you install the Daheng Galaxy SDK, it creates a folder at:
`C:\Program Files\Daheng Imaging\GalaxySDK\`

### 2.1 Top-Level Folders

- **APIDLL** — Contains the actual .dll runtime files. When your compiled .exe runs, Windows looks for these. You may need to copy them next to your .exe or add this folder to your PATH.
- **Development/Samples** — Contains sample projects in C++, C#, Python and more. The GxMultiCam and GxSimpleGrab samples are especially useful for learning.
- **Development/C++ SDK** — Contains everything you need to write and compile code: header files and lib files.
- **Drivers** — USB camera drivers. Must be installed before cameras will be recognized.
- **Tools** — GalaxyView.exe lives here. This is a GUI tool to test your cameras without writing any code. Very useful for checking camera connectivity and finding exact feature names (Width, Height, ExposureTime etc.).

### 2.2 The C++ SDK Folder

This is the folder you reference in CMakeLists.txt:
`C:\Program Files\Daheng Imaging\GalaxySDK\Development\C++ SDK\`

Inside it:
- `inc/` — all the header files. The key file is `GalaxyIncludes.h` which includes all others automatically.
- `lib/x64/` — contains `GxIAPICPPEx.lib` (the one we use on 64-bit Windows)
- `lib/x86/` — contains the 32-bit version (we do not use this)

NOTE: Always use the x64 folder on a 64-bit Windows system. Mixing 32-bit and 64-bit will cause linking errors.

---

## Chapter 3: CMakeLists.txt — Original and Updated

### 3.1 Original CMakeLists.txt (Daheng SDK only)

This was the starting version — Daheng SDK only, no display capability:

```cmake
cmake_minimum_required(VERSION 3.15)
project(MultiCameraApp)
set(CMAKE_CXX_STANDARD 17)

set(DAHENG_SDK_DIR "C:/Program Files/Daheng Imaging/GalaxySDK/Development/C++ SDK")

include_directories(${DAHENG_SDK_DIR}/inc)
link_directories(${DAHENG_SDK_DIR}/lib/x64)

add_executable(MultiCameraApp camera.cpp)

target_link_libraries(MultiCameraApp GxIAPICPPEx)
```

### 3.2 Updated CMakeLists.txt (Daheng SDK + OpenCV)

This is the current working version with OpenCV added for live display:

```cmake
cmake_minimum_required(VERSION 3.15)
project(MultiCameraApp)
set(CMAKE_CXX_STANDARD 17)

# Daheng SDK
set(DAHENG_SDK_DIR "C:/Program Files/Daheng Imaging/GalaxySDK/Development/C++ SDK")
include_directories(${DAHENG_SDK_DIR}/inc)
link_directories(${DAHENG_SDK_DIR}/lib/x64)

# OpenCV
set(OPENCV_DIR "C:/opencv/build")
include_directories(${OPENCV_DIR}/include)
link_directories(${OPENCV_DIR}/x64/vc16/lib)

add_executable(MultiCameraApp camera.cpp)

target_link_libraries(MultiCameraApp
    GxIAPICPPEx
    opencv_world4120
)
```

### 3.3 What Changed and Why

**Added OpenCV include directory:**
```cmake
include_directories(${OPENCV_DIR}/include)
```
Without this, the compiler cannot find `<opencv2/opencv.hpp>` and will error with "No such file or directory".

**Added OpenCV lib directory:**
```cmake
link_directories(${OPENCV_DIR}/x64/vc16/lib)
```
Without this, the linker cannot find opencv_world4120.lib.

**Added opencv_world4120 to target_link_libraries:**
```cmake
target_link_libraries(MultiCameraApp
    GxIAPICPPEx
    opencv_world4120
)
```
OpenCV ships all its functionality in one "world" library. The number 4120 is the version (4.12.0). This must match exactly what is in your lib folder — check with:
```
dir C:\opencv\build\x64\vc16\lib\
```

### 3.4 Line by Line Explanation (Original Lines)

**`cmake_minimum_required(VERSION 3.15)`**
Declares the minimum CMake version needed. You have 4.1.2 so this is satisfied.

**`project(MultiCameraApp)`**
Names the project. This name becomes your .exe filename and must appear in `add_executable()` and `target_link_libraries()`.

**`set(CMAKE_CXX_STANDARD 17)`**
Tells the compiler to use C++17. Required for `std::mutex`, `std::lock_guard`, and other modern features used in the multi-camera callback code.

**`set(DAHENG_SDK_DIR ...)`**
A custom variable storing the SDK path. Defined once, reused multiple times. If the SDK moves, change only this one line.

NOTE: CMake always uses forward slashes `/` in paths, even on Windows. Backslashes `\` are escape characters in CMake strings and will cause syntax errors.

**`include_directories(...)`**
Tells the compiler where to search for header files. Every `#include` statement searches these directories.

**`link_directories(...)`**
Tells the linker where to search for `.lib` files.

**`add_executable(MultiCameraApp camera.cpp)`**
The core instruction: build an executable from this source file.

**`target_link_libraries(MultiCameraApp GxIAPICPPEx)`**
Links the SDK library. CMake automatically adds `.lib` extension on Windows.

---

## Chapter 4: camera.cpp — Complete Evolution

### 4.1 Starting Version (camera.cpp — enumerate only)

This was the first working version — finds cameras, prints their info, opens and closes them:

```cpp
#include "GalaxyIncludes.h"
#include <iostream>
#include <vector>

int main()
{
    try
    {
        IGXFactory::GetInstance().Init();
        std::cout << "SDK Initialized." << std::endl;

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

        for (int i = 0; i < camCount; ++i)
        {
            std::cout << "Camera " << i << ": "
                      << vecDeviceInfo[i].GetModelName()
                      << " | SN: " << vecDeviceInfo[i].GetSN()
                      << std::endl;
        }

        std::vector<CGXDevicePointer> cameras(camCount);
        for (int i = 0; i < camCount; ++i)
        {
            cameras[i] = IGXFactory::GetInstance().OpenDeviceBySN(
                vecDeviceInfo[i].GetSN(), GX_ACCESS_EXCLUSIVE
            );
            std::cout << "Opened camera " << i << std::endl;
        }

        for (int i = 0; i < camCount; ++i)
        {
            cameras[i]->Close();
            std::cout << "Closed camera " << i << std::endl;
        }

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
```

### 4.2 Final Version (camera.cpp — live multi-camera display)

This is the current working version with full streaming, display, and clean shutdown:

```cpp
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

        for (int i = 0; i < camCount; ++i)
        {
            cameras[i] = IGXFactory::GetInstance().OpenDeviceBySN(
                vecDeviceInfo[i].GetSN(), GX_ACCESS_EXCLUSIVE
            );

            featureControls[i] = cameras[i]->GetRemoteFeatureControl();

            // Set full resolution
            featureControls[i]->GetIntFeature("Width")->SetValue(2600);
            featureControls[i]->GetIntFeature("Height")->SetValue(2160);
            featureControls[i]->GetIntFeature("OffsetX")->SetValue(0);
            featureControls[i]->GetIntFeature("OffsetY")->SetValue(0);

            featureControls[i]->GetEnumFeature("AcquisitionMode")->SetValue("Continuous");

            streams[i] = cameras[i]->OpenStream(0);

            callbacks[i] = std::make_shared<CCaptureHandler>(i);
            streams[i]->RegisterCaptureCallback(callbacks[i].get(), nullptr);

            streams[i]->StartGrab();
            featureControls[i]->GetCommandFeature("AcquisitionStart")->Execute();

            std::cout << "Camera " << i << " started." << std::endl;
        }

        // Step 5: Display loop
        std::cout << "Press Q to quit." << std::endl;

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
            if (cv::getWindowProperty("Camera 0 [" + std::string(vecDeviceInfo[0].GetSN()) + "]",
                                      cv::WND_PROP_VISIBLE) < 1)
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
```

---

## Chapter 5: Code Changes Explained — What We Added and Why

### 5.1 New Includes

```cpp
#include <opencv2/opencv.hpp>
#include <mutex>
```

- `opencv2/opencv.hpp` — the OpenCV master header. Gives access to `cv::Mat`, `cv::imshow`, `cv::waitKey`, `cv::flip`, `cv::cvtColor`. Without this the compiler does not know what any of those mean.
- `mutex` — C++ standard library for thread synchronization. The camera callback fires on a background thread while the display loop runs on the main thread. Without a mutex protecting the shared frame data, both threads could read/write it simultaneously, causing corrupted frames or crashes.

### 5.2 The CameraFrame Struct

```cpp
struct CameraFrame
{
    cv::Mat frame;
    std::mutex mtx;
    bool hasNewFrame = false;

    CameraFrame() = default;
    CameraFrame(CameraFrame&& other) noexcept
        : frame(std::move(other.frame)), hasNewFrame(other.hasNewFrame) {}
};
```

This struct bundles a frame, its mutex, and a flag together per camera. The mutex ensures that the callback thread writing a new frame and the display thread reading it never do so at the same time.

**Why the move constructor was needed (and the error it fixed):**

`std::mutex` cannot be copied — it is a hardware synchronization primitive that has no meaningful copy semantics. When you put a `std::mutex` inside a struct, that struct also becomes non-copyable automatically.

`std::vector::resize()` internally needs to move or copy its elements to a new memory location when it grows. Without an explicit move constructor, the compiler tried to copy `CameraFrame`, failed because of the mutex, and produced this error:

```
error C2280: 'CameraFrame::CameraFrame(const CameraFrame &)': attempting to reference
a deleted function
```

The fix was to explicitly define a move constructor that moves the `cv::Mat` and copies the `bool`, but skips the mutex (the new location gets a fresh default-constructed mutex, which is correct — mutexes cannot and should not be transferred):

```cpp
CameraFrame(CameraFrame&& other) noexcept
    : frame(std::move(other.frame)), hasNewFrame(other.hasNewFrame) {}
```

The `noexcept` keyword tells the compiler this move will not throw exceptions, which allows `std::vector` to use it safely during reallocation.

### 5.3 The CCaptureHandler Callback Class

```cpp
class CCaptureHandler : public ICaptureEventHandler
{
public:
    int m_index;
    CCaptureHandler(int index) : m_index(index) {}

    void DoOnImageCaptured(CImageDataPointer& imgPtr, void* pUserParam) override
    { ... }
};
```

This class registers with the SDK stream. Every time the camera delivers a new frame, the SDK calls `DoOnImageCaptured()` automatically on a background thread. The `m_index` tells it which camera this handler belongs to, so it stores the frame in the correct slot in `g_frames`.

### 5.4 ConvertToRGB24 — The Wrong Call and the Right Call

**Original broken code:**
```cpp
void* pRGBBuffer = malloc(bufferSize);
imgPtr->ConvertToRGB24(GX_BIT_0_7, GX_RAW2RGB_NEIGHBOUR, true, pRGBBuffer, bufferSize);
```

**Error it produced:**
```
error C2660: 'IImageData::ConvertToRGB24': function does not take 5 arguments
```

**Why it was wrong:** The Daheng SDK's `ConvertToRGB24` manages its own internal buffer. It takes 3 arguments (bit depth, conversion method, flip flag) and returns a `void*` pointer to its own buffer. It does not accept an external buffer from the caller.

Passing 5 arguments (including an external buffer and its size) is the API signature of a different SDK or an older version. The actual SDK header at line 107 shows:

```cpp
virtual void* ConvertToRGB24(GX_VALID_BIT_LIST emValidBits,
                              GX_BAYER_CONVERT_TYPE_LIST emConvertType,
                              bool bFlip) = 0;
```

**Fixed code:**
```cpp
void* pRGBBuffer = imgPtr->ConvertToRGB24(GX_BIT_0_7, GX_RAW2RGB_NEIGHBOUR, true);
```

Two important consequences:
1. No `malloc` needed — the SDK owns the buffer
2. No `free` needed — do NOT call `free(pRGBBuffer)`. The SDK allocated this memory and will free it. Calling `free` on it would be a double-free and crash the program.

### 5.5 Color Channel Order — RGB to BGR

```cpp
cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGB2BGR);
```

The Daheng SDK outputs pixel data in RGB order (Red, Green, Blue). OpenCV internally expects BGR order (Blue, Green, Red) — this is a historical quirk of OpenCV. Without this conversion, red objects appear blue and blue objects appear red.

### 5.6 Flipping the Image

```cpp
cv::flip(bgrMat, bgrMat, 0);
```

The camera was physically mounted so that the image arrived upside down. `cv::flip` with a flip code of `0` flips vertically (around the horizontal axis). Flip code `1` would flip horizontally. Flip code `-1` flips both.

This is a one-line fix but the cause is important to understand: it is not a software bug, it is a physical mounting orientation. If you remount the camera the right way up, remove this line.

### 5.7 Thread-Safe Frame Storage

```cpp
{
    std::lock_guard<std::mutex> lock(g_frames[m_index].mtx);
    g_frames[m_index].frame = bgrMat.clone();
    g_frames[m_index].hasNewFrame = true;
}
```

`std::lock_guard` is a RAII wrapper — it locks the mutex when created and unlocks it automatically when it goes out of scope (the closing `}`). This is important: if you forget to unlock a mutex the program deadlocks. With lock_guard, the unlock is guaranteed even if an exception is thrown.

`bgrMat.clone()` creates a deep copy of the pixel data. Without clone(), `frame` would hold a reference to `bgrMat`'s data, which is local to this callback call and would be destroyed when the function returns, leaving a dangling pointer.

### 5.8 The Display Loop and Quit Behaviour

**Original (broken) quit:**
```cpp
int key = cv::waitKey(1);
if (key == 'q' || key == 'Q' || key == 27)
    break;
```

**Problems:**
1. `cv::waitKey` returns an int that can have high bits set on some platforms. `& 0xFF` masks to just the low byte to get the actual ASCII key code reliably.
2. If you closed the window with the X button, the program kept running in the background with no way to quit.

**Fixed quit:**
```cpp
int key = cv::waitKey(1) & 0xFF;
if (key == 'q' || key == 'Q' || key == 27)
    break;
if (cv::getWindowProperty("Camera 0 [" + std::string(vecDeviceInfo[0].GetSN()) + "]",
                          cv::WND_PROP_VISIBLE) < 1)
    break;
```

`cv::getWindowProperty` with `WND_PROP_VISIBLE` returns 1.0 if the window is open and visible, less than 1 if it has been closed. This gives the X button the same effect as pressing Q.

### 5.9 Full Resolution Setting

```cpp
featureControls[i]->GetIntFeature("Width")->SetValue(2600);
featureControls[i]->GetIntFeature("Height")->SetValue(2160);
featureControls[i]->GetIntFeature("OffsetX")->SetValue(0);
featureControls[i]->GetIntFeature("OffsetY")->SetValue(0);
```

Camera sensors support ROI (Region of Interest) — you can tell the camera to read only a portion of the sensor to save bandwidth and increase frame rate. Setting Width and Height to the sensor's maximum (2600x2160) uses the full sensor.

OffsetX and OffsetY set where the ROI starts within the sensor. Setting both to 0 means start from the top-left corner. This ensures the ROI covers the full sensor.

**Important note:** Some cameras require Width and Height to be multiples of 8 or 16. If you get an error when setting these values, open GalaxyView, find the Width feature, and check its Increment property. Use a value that is both within the maximum and a multiple of that increment.

### 5.10 Clean Shutdown — Order Matters

```cpp
featureControls[i]->GetCommandFeature("AcquisitionStop")->Execute();
streams[i]->StopGrab();
streams[i]->UnregisterCaptureCallback();
streams[i]->Close();
cameras[i]->Close();
```

The shutdown order must be the exact reverse of startup:
1. Tell the camera hardware to stop sending data (`AcquisitionStop`)
2. Tell the SDK stream to stop receiving data (`StopGrab`)
3. Unregister the callback so no new frames can arrive
4. Close the stream
5. Close the camera device

Doing these out of order can cause the callback to fire after the stream is closed (crash), or leave the camera hardware in a state where the next run cannot open it (you would see "camera already in use" errors).

---

## Chapter 6: All Errors Encountered and How We Fixed Them

### Error 1: ConvertToRGB24 does not take 5 arguments

```
error C2660: 'IImageData::ConvertToRGB24': function does not take 5 arguments
note: while trying to match the argument list '(GX_VALID_BIT_LIST, GX_BAYER_CONVERT_TYPE_LIST, bool, void *, size_t)'
```

**Cause:** The function was called with 5 arguments (including an external buffer and its size), but the SDK's actual signature only takes 3 arguments and returns the buffer pointer itself.

**Fix:** Remove the external buffer allocation and use the return value instead:
```cpp
// WRONG
void* pRGBBuffer = malloc(bufferSize);
imgPtr->ConvertToRGB24(GX_BIT_0_7, GX_RAW2RGB_NEIGHBOUR, true, pRGBBuffer, bufferSize);
// ...
free(pRGBBuffer);

// CORRECT
void* pRGBBuffer = imgPtr->ConvertToRGB24(GX_BIT_0_7, GX_RAW2RGB_NEIGHBOUR, true);
// no free() — SDK owns this buffer
```

---

### Error 2: CameraFrame copy constructor deleted due to std::mutex

```
error C2280: 'CameraFrame::CameraFrame(const CameraFrame &)': attempting to reference a deleted function
note: 'std::mutex::mutex(const std::mutex &)': function was explicitly deleted
```

**Cause:** `std::mutex` is not copyable. When a struct contains a mutex, it also becomes non-copyable. `std::vector::resize()` tried to copy `CameraFrame` objects when reallocating, which failed.

**Fix:** Add an explicit move constructor so the vector can move elements instead of copying them:
```cpp
CameraFrame(CameraFrame&& other) noexcept
    : frame(std::move(other.frame)), hasNewFrame(other.hasNewFrame) {}
```
The mutex member is not transferred — the moved-into object gets a fresh default mutex, which is correct behaviour.

---

### Error 3: Program exits immediately with no output

**Symptom:** Running `build\MultiCameraApp.exe` returned to the prompt instantly with zero output — not even "SDK Initialized."

**Cause:** A required DLL was missing. Windows silently terminates any program that cannot load all its DLL dependencies at startup. The program never even reached the first line of `main()`.

This happened twice:
1. First time: OpenCV DLL was missing after a fresh build
2. The Daheng SDK DLLs were fine because they were already in the system PATH from the SDK installer

**Diagnosis:** Run with `& pause` to keep the console open. If there is truly zero output (not even the first `std::cout`), the issue is a missing DLL, not a logic error.

**Fix:** Copy the missing DLL next to the .exe:
```
copy "C:\opencv\build\x64\vc16\bin\opencv_world4120.dll" build\
```

**Why this happens after every clean build:** When you delete the `build\` folder and rebuild, the new `build\` folder is empty — the DLL you copied before is gone. You must copy it again after every clean build.

---

### Error 4: NMake not found — "no such file or directory"

```
CMake Error: Generator: build tool execution failed, command was: nmake -f Makefile /nologo
no such file or directory
```

**Cause:** The terminal session did not have the MSVC environment activated. NMake is a tool that comes with Visual Studio but is only available after running the vcvars64.bat activation script. In a plain PowerShell or Command Prompt, `nmake` does not exist.

**Fix:** Always activate MSVC before building. Open a new terminal and run:
```
cmd /k "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
```
Then navigate to the project and build. This must be done every time you open a new terminal.

---

### Error 5: CMakeCache conflict — generator does not match

```
CMake Error: Generator: Visual Studio 17 2022
Does not match the generator used previously: NMake Makefiles
Either remove the CMakeCache.txt file and CMakeFiles directory or choose a different binary directory.
```

**Cause:** CMake stores the generator choice in `build\CMakeCache.txt`. Once a build folder is configured with one generator, CMake refuses to reconfigure it with a different one.

**Fix:** Delete the entire build folder and start fresh:
```
rmdir /s /q build
cmake -S . -B build -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release
```

**Why this is the right approach:** Trying to modify CMakeCache.txt manually is fragile. Deleting the build folder guarantees a clean state. You lose the previous build output but that is acceptable — it will be fully rebuilt.

---

### Error 6: Visual Studio generator not found

```
CMake Error: Generator Visual Studio 17 2022 could not find any instance of Visual Studio.
```

**Cause:** The installed Visual Studio is version 2026 (internal version 18). The CMake generator "Visual Studio 17 2022" looks for VS 2022 (internal version 17), which is not installed. CMake 4.1.2 does not yet include a generator entry for VS 2026.

**Fix:** Use the NMake Makefiles generator instead, which works directly with the MSVC compiler tools regardless of Visual Studio version number:
```
cmake -S . -B build -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release
```
NMake comes with any Visual Studio installation and works with the MSVC compiler activated via vcvars64.bat.

---

### Error 7: Image feed upside down

**Symptom:** The camera feed displayed correctly but was vertically flipped.

**Cause:** The camera was physically mounted upside down. The sensor delivers pixel data row by row from top to bottom as it sees them, so an upside-down camera produces an upside-down image in software.

**Fix:** Add a vertical flip after color conversion:
```cpp
cv::flip(bgrMat, bgrMat, 0);  // 0 = flip vertically
```

This is a software workaround. The permanent fix would be to mount the camera the right way up and remove this line.

---

### Error 8: Cannot quit the application

**Symptom:** Pressing Q had no effect. Closing the window with the X button left the process running.

**Cause 1 — Key masking:** `cv::waitKey()` returns an int where on some platforms the high bits carry modifier key state. Comparing the raw value against `'q'` (ASCII 113) can fail if those high bits are set.

**Fix:** Mask with `& 0xFF` to isolate only the key code:
```cpp
int key = cv::waitKey(1) & 0xFF;
```

**Cause 2 — No window close detection:** The original code only checked keyboard input. Closing the OpenCV window with the X button does not generate a keyboard event — it changes the window's visibility property. Without checking that property, the program kept running.

**Fix:** Add a window property check:
```cpp
if (cv::getWindowProperty("Camera 0 [" + std::string(vecDeviceInfo[0].GetSN()) + "]",
                          cv::WND_PROP_VISIBLE) < 1)
    break;
```

---

## Chapter 7: Environment Setup Checklist

Every time you start a new camera project, run through this checklist.

### 7.1 Verify the Compiler
Open the x64 Native Tools Command Prompt and run:
```
cl
```
Expected: `Microsoft (R) C/C++ Optimizing Compiler Version 19.xx for x64`

### 7.2 Verify CMake
```
cmake --version
```
Expected: `cmake version 4.x.x or higher`

### 7.3 Verify the SDK Headers
```
dir "C:\Program Files\Daheng Imaging\GalaxySDK\Development\C++ SDK\inc"
```
Expected: You should see `GalaxyIncludes.h` and many other .h files.

### 7.4 Verify the SDK Library
```
dir "C:\Program Files\Daheng Imaging\GalaxySDK\Development\C++ SDK\lib\x64"
```
Expected: You should see `GxIAPICPPEx.lib`.

### 7.5 Verify OpenCV
```
dir C:\opencv\build\x64\vc16\bin\opencv_world*.dll
```
Expected: `opencv_world4120.dll`

### 7.6 Verify Camera Connection
Open GalaxyView from the Daheng Tools folder. If your cameras appear in the device list, the drivers are working and the cameras are connected correctly. Use GalaxyView to find exact feature names (Width, Height, ExposureTime, Gain etc.) for your specific camera model.

---

## Chapter 8: The SDK Architecture

### 8.1 The Object Hierarchy

```
IGXFactory
    |-- enumerates devices
    |-- opens devices --> IGXDevice
                             |-- controls camera settings --> IGXFeatureControl
                             |-- opens streams --> IGXStream
                                                       |-- receives frames --> IImageData
```

### 8.2 IGXFactory
The top-level singleton. Responsible for initializing the library, discovering cameras, and opening them. Every program starts and ends here with `Init()` and `Uninit()`.

### 8.3 IGXDevice
Represents one physical camera. You get this from `IGXFactory::OpenDeviceBySN()`. Through this object you access feature control and open the image stream.

### 8.4 IGXFeatureControl
Controls all camera parameters: exposure, gain, resolution, frame rate, pixel format, and hundreds more. You get it from `IGXDevice::GetRemoteFeatureControl()`. Features are accessed by name string: `"ExposureTime"`, `"Gain"`, `"Width"`, `"Height"`, etc.

Feature types and their getter methods:
- Integer features (Width, Height, OffsetX): `GetIntFeature("Name")->SetValue(val)`
- Enum features (AcquisitionMode, PixelFormat): `GetEnumFeature("Name")->SetValue("Value")`
- Command features (AcquisitionStart, AcquisitionStop): `GetCommandFeature("Name")->Execute()`
- Float features (ExposureTime, Gain): `GetFloatFeature("Name")->SetValue(val)`

### 8.5 IGXStream
Manages the flow of image data from the camera to your program. You get it from `IGXDevice::OpenStream(0)`. The stream handles buffering, callbacks, and frame acquisition.

### 8.6 IImageData
Represents a single captured frame. Contains the raw pixel buffer, width, height, pixel format, frame ID, and timestamp.

### 8.7 Smart Pointers
All SDK objects end in Pointer: `CGXDevicePointer`, `CGXStreamPointer` etc. These are smart pointers — they automatically manage memory and release resources when they go out of scope.

---

## Quick Reference Card

### Build Commands (run in x64 Native Tools Command Prompt)
```
cd C:\Git\TARA-camera
cmake -S . -B build -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build
copy "C:\opencv\build\x64\vc16\bin\opencv_world4120.dll" build\
build\MultiCameraApp.exe
```

### Rebuild After Code Changes (no clean needed)
```
cmake --build build
```

### Clean Build (after CMake generator errors or cache conflicts)
```
rmdir /s /q build
cmake -S . -B build -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release
cmake --build build
copy "C:\opencv\build\x64\vc16\bin\opencv_world4120.dll" build\
```

### Activate MSVC in Any Terminal
```
cmd /k "C:\Program Files\Microsoft Visual Studio\18\Community\VC\Auxiliary\Build\vcvars64.bat"
```

### Key Paths
```
SDK Headers : C:/Program Files/Daheng Imaging/GalaxySDK/Development/C++ SDK/inc/
SDK Library : C:/Program Files/Daheng Imaging/GalaxySDK/Development/C++ SDK/lib/x64/GxIAPICPPEx.lib
SDK DLLs    : C:/Program Files/Daheng Imaging/GalaxySDK/APIDLL/
SDK Samples : C:/Program Files/Daheng Imaging/GalaxySDK/Development/Samples/C++/
OpenCV DLL  : C:/opencv/build/x64/vc16/bin/opencv_world4120.dll
OpenCV Lib  : C:/opencv/build/x64/vc16/lib/opencv_world4120.lib
OpenCV Inc  : C:/opencv/build/include/
```

### SDK Initialization Pattern (Always)
```cpp
IGXFactory::GetInstance().Init();    // First line always
// ... your code ...
IGXFactory::GetInstance().Uninit();  // Last line always
```

### Camera Access Modes
```cpp
GX_ACCESS_EXCLUSIVE  // Only your program can use the camera (recommended)
GX_ACCESS_READONLY   // Multiple programs can read but not control
```

### cv::flip Codes
```cpp
cv::flip(src, dst, 0);   // flip vertically   (upside down fix)
cv::flip(src, dst, 1);   // flip horizontally (mirror fix)
cv::flip(src, dst, -1);  // flip both
```

---

*End of Guide*
