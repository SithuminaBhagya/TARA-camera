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

# Zstd: downloaded and built on first configure via FetchContent.
# No manual install needed — CMake clones the release source and builds
# libzstd_static as part of the project.
include(FetchContent)
set(ZSTD_BUILD_STATIC ON  CACHE BOOL "" FORCE)
set(ZSTD_BUILD_SHARED OFF CACHE BOOL "" FORCE)
set(ZSTD_BUILD_PROGRAMS OFF CACHE BOOL "" FORCE)
set(ZSTD_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(ZSTD_MULTITHREAD_SUPPORT OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
    zstd
    URL https://github.com/facebook/zstd/releases/download/v1.5.6/zstd-1.5.6.tar.gz
    URL_HASH SHA256=8c29e06cf42aacc1eafc4077ae2ec6c6fcb96a626157e0593d5e82a34fd403c1
    SOURCE_SUBDIR build/cmake
)
FetchContent_MakeAvailable(zstd)
# Zstd headers live alongside the cloned source
include_directories(${zstd_SOURCE_DIR}/lib)

add_executable(MultiCameraApp camera.cpp)
target_link_libraries(MultiCameraApp
    GxIAPICPPEx
    opencv_world4120
)

add_executable(CamDetails camDetails.cpp)
target_link_libraries(CamDetails
    GxIAPICPPEx
)

add_executable(SaveFrames saveFrames.cpp)
target_link_libraries(SaveFrames
    GxIAPICPPEx
    libzstd_static
)

add_executable(Playback playback.cpp)
target_link_libraries(Playback
    opencv_world4120
    libzstd_static
)

add_executable(LiveFeed liveFeed.cpp)
target_link_libraries(LiveFeed
    GxIAPICPPEx
    opencv_world4120
)
