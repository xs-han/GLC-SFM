//
// Created by xushen on 10/13/17.
//

#ifndef MV_SLAM_SLAM_H
#define MV_SLAM_SLAM_H

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "MapPoint.h"
#include "KeyFrame.h"
#include "VideoStream.h"
#include "ImageStream.h"

using namespace cv;
using namespace std;

class SLAM {
public:
    Mat cameraMartix;
    cv::Ptr<cv::Feature2D> detector;
    FeatureMatcher matcher;

    Mat distortionCoefficient;
    vector<MapPoint *> pointClouds;
    vector<KeyFrame *> allKeyFrames;

    MediaStream * ms;

    explicit SLAM(string settingFile = "../cfg/setting.xml");

    void setCameraIntrinsicParams(string calibFile);

    void process();

    bool isFrameKey(const KeyFrame &refKf, const Mat & newFrame);

    void initialize();

    void track();

    void localmap();

    void drawMap();

    void undistortFrame(Mat &input, Mat &output);
};


#endif //MV_SLAM_SLAM_H
