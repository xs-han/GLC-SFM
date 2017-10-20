cmake_minimum_required(VERSION 3.5)
project(MV_SLAM)

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS}, ${Eigen_INCLUDE_DIRS} "/usr/include/eigen3")
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp
        SLAM.h SLAM.cpp
        MapPoint.h
        KeyFrame.h KeyFrame.cpp
        MediaStream.h
        VideoStream.h VideoStream.cpp
        ImageStream.h ImageStream.cpp
        FeatureMatcher.h FeatureMatcher.cpp MapPaint.cpp MapPaint.h)
add_executable(MV_SLAM ${SOURCE_FILES})
target_link_libraries(MV_SLAM ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Eigen_LIBRARIES})