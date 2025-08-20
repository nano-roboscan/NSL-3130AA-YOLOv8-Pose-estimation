cmake_minimum_required(VERSION 3.5)
project(yolov8)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)
find_package(OpenMP REQUIRED)
if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


find_path(NSL_LIB_INCLUDE_DIR NAMES nanolib.h PATHS nsl_lib/include REQUIRED)
#find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/linux-7.5/shared REQUIRED)
find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/aarch-7.5/shared REQUIRED)
#find_library(NSL_LIB_PATH NAMES nanolib PATHS nsl_lib/lib/windows/shared REQUIRED)


set(PROJECT_SOURCES
        main.cpp videoSource.cpp
        yolo/YoloDet.cpp
        yolo/YoloPose.cpp
        tools/ImageTools.cpp
        tools/timeCheck.cpp)

add_executable(yolov8 ${PROJECT_SOURCES})

target_include_directories(yolov8 PRIVATE ./ ./yolo ./tools ${NSL_LIB_INCLUDE_DIR})
target_link_libraries(yolov8 ${NSL_LIB_PATH} ${OpenCV_LIBS} Threads::Threads)

