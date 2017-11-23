cmake_minimum_required(VERSION 3.5)
project(GLC_SFM)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/g2o/cmake_modules ${PROJECT_SOURCE_DIR}/Pangolin/CMakeModules)

find_package(OpenCV REQUIRED)
find_package(Cholmod REQUIRED)

add_subdirectory(g2o)
add_subdirectory(DBow3)
add_subdirectory(Pangolin)

SET(G2O_LIBS
        core
        types_slam3d
        solver_csparse
        stuff
        csparse_extension
        types_sba
        )

include_directories(${OpenCV_INCLUDE_DIRS}, "/usr/include/eigen3"
        ${PROJECT_SOURCE_DIR}/g2o ${PROJECT_BINARY_DIR}/g2o
        ${PROJECT_SOURCE_DIR}/Pangolin/include ${PROJECT_BINARY_DIR}/Pangolin/src/include
        ${PROJECT_SOURCE_DIR}/DBow3/src
        ${CHOLMOD_INCLUDE_DIR})

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

add_executable(GLC_SFM ${SOURCE_FILES})
target_link_libraries(GLC_SFM ${OpenCV_LIBS} pangolin ${CHOLMOD_LIBRARIES} ${G2O_LIBS} DBoW3)

add_executable(calibration util/camera_calibration.cpp)
target_link_libraries(calibration ${OpenCV_LIBS})

add_executable(create_voc_step0 util/create_voc_step0.cpp)
target_link_libraries(create_voc_step0 ${OpenCV_LIBS} DBoW3)

add_executable(create_voc_step1 util/create_voc_step1.cpp)
target_link_libraries(create_voc_step1 ${OpenCV_LIBS} DBoW3)

add_executable(compute_voc_adjecent_dis util/compute_voc_adjecent_dis.cpp)
target_link_libraries(compute_voc_adjecent_dis ${OpenCV_LIBS} DBoW3)

add_executable(compute_bow_with_angle util/compute_bow_with_angle.cpp VideoStream.cpp VideoStream.h)
target_link_libraries(compute_bow_with_angle ${OpenCV_LIBS} DBoW3)