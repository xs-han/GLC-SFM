cmake_minimum_required(VERSION 3.5)
project(MV_SLAM)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/g2o/cmake_modules )

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
#find_package(Eigen3 REQUIRED)
find_package(Cholmod REQUIRED)

add_subdirectory(g2o)
add_subdirectory(DBow3)

SET(G2O_LIBS
        core
        types_slam3d
        solver_csparse
        stuff
        csparse_extension
        types_sba
        )

include_directories(${OpenCV_INCLUDE_DIRS}, "/usr/include/eigen3"
        ${PROJECT_SOURCE_DIR}/g2o ${PROJECT_BINARY_DIR}/g2o ${CHOLMOD_INCLUDE_DIR})

set(SOURCE_FILES main.cpp
        SLAM.h SLAM.cpp
        MapPoint.h MapPoint.cpp
        KeyFrame.h KeyFrame.cpp
        MediaStream.h
        VideoStream.h VideoStream.cpp
        ImageStream.h ImageStream.cpp
        FeatureMatcher.h FeatureMatcher.cpp
        MapPaint.cpp MapPaint.h
        Optimizer.cpp Optimizer.h
        )
add_executable(MV_SLAM ${SOURCE_FILES})
target_link_libraries(MV_SLAM ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${CHOLMOD_LIBRARIES} ${G2O_LIBS} DBoW3)