cmake_minimum_required(VERSION 3.5)
project(MV_SLAM)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp
        MapPoint.h
        KeyFrame.h
        VideoStream.h ImageStream.h SLAM.cpp SLAM.h MediaStream.h VideoStream.cpp ImageStream.cpp
        FeatureMatcher.h FeatureMatcher.cpp KeyFrame.cpp)
add_executable(MV_SLAM ${SOURCE_FILES})
target_link_libraries(MV_SLAM ${OpenCV_LIBS})