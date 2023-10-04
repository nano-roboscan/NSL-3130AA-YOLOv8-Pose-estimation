cmake_minimum_required(VERSION 3.5)
project(yolov8)

set(CMAKE_CXX_STANDARD 11)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

set(PROJECT_SOURCES
        main.cpp main.h
        yolo/YoloDet.cpp yolo/YoloDet.h
        yolo/YoloPose.cpp yolo/YoloPose.h
        tools/ImageTools.cpp tools/ImageTools.h
		tools/timecheck.cpp tools/timecheck.h
        lidar/lens_transform.cpp lidar/lens_transform.h
        lidar/NSL3130AA.cpp lidar/NSL3130AA.h
        lidar/videoSource.cpp lidar/videoSource.h
        lidar/NSLFrame.h
        )

add_executable(yolov8 ${PROJECT_SOURCES})

target_include_directories(yolov8 PRIVATE ./ ./yolo ./tools ./lidar)
target_link_libraries(yolov8 ${OpenCV_LIBS} Threads::Threads)

