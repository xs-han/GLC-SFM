cmake_minimum_required(VERSION 3.5)
project(MV_SLAM)
set(G2O_ROOT ~/.local)
list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/g2o/cmake_modules )

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)
find_package( Cholmod REQUIRED)

SET(G2O_LIBS
        ~/.local/lib/libg2o_core.so
        ~/.local/lib/libg2o_types_slam3d.so
        ~/.local/lib/libg2o_solver_csparse.so
        ~/.local/lib/libg2o_stuff.so
        ~/.local/lib/libg2o_csparse_extension.so
        ~/.local/lib/libg2o_types_sba.so
        )

include_directories(${OpenCV_INCLUDE_DIRS}, ${Eigen_INCLUDE_DIRS} "/usr/include/eigen3"
        ~/.local/include ${CHOLMOD_INCLUDE_DIR})
set(CMAKE_CXX_STANDARD 11)

set(SOURCE_FILES main.cpp
        SLAM.h SLAM.cpp
        MapPoint.h
        KeyFrame.h KeyFrame.cpp
        MediaStream.h
        VideoStream.h VideoStream.cpp
        ImageStream.h ImageStream.cpp
        FeatureMatcher.h FeatureMatcher.cpp MapPaint.cpp MapPaint.h Optimizer.cpp Optimizer.h)
add_executable(MV_SLAM ${SOURCE_FILES})
target_link_libraries(MV_SLAM ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Eigen_LIBRARIES} ${CHOLMOD_LIBRARIES} ${G2O_LIBS} )