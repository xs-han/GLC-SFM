//
// Created by xushen on 10/13/17.
//

#ifndef MV_SLAM_SLAM_H
#define MV_SLAM_SLAM_H

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "MapPoint.hpp"
#include "KeyFrame.hpp"
#include "VideoStream.h"
#include "ImageStream.h"

using namespace cv;
using namespace std;

class SLAM {
public:
    static Mat cameraMartix;
    Mat distortionCoefficient;
    vector<MapPoint *> pointClouds;
    vector<KeyFrame *> allKeyFrames;

    MediaStream * ms;

    explicit SLAM(string settingFile = "../xml/setting.xml");

    void setCameraIntrinsicParams(string calibFile);

    void process();

    void initialize();

    void track();

    void localmap();

    void drawMap();

};


#endif //MV_SLAM_SLAM_H
