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

add_executable(CamDetails camDetails.cpp)
target_link_libraries(CamDetails
    GxIAPICPPEx
)

add_executable(SaveFrames saveFrames.cpp)
target_link_libraries(SaveFrames
    GxIAPICPPEx
    opencv_world4120
)

add_executable(Playback playback.cpp)
target_link_libraries(Playback
    opencv_world4120
)