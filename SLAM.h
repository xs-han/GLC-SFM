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
#include "MapPaint.h"

using namespace cv;
using namespace std;

class SLAM {
public:
    bool rectified;
    int imageScale;
    Mat cameraMatrix;
    Mat m1, m2, newCameraMatrix;
    cv::Ptr<cv::Feature2D> detector;
    FeatureMatcher matcher;

    Mat distortionCoefficient;
    vector<MapPoint *> pointClouds;
    vector<KeyFrame *> allKeyFrames;

    MediaStream * ms;
    MapPaint mPaint;

    explicit SLAM(string settingFile = "../cfg/setting.xml");

    void setCameraIntrinsicParams(string calibFile);

    void process();

    void initialize();

    void undistortFrame(Mat &input, Mat &output);

    void track(KeyFrame & k, const vector <DMatch> & matches);

    void localmap(KeyFrame & k, const vector <DMatch> & matches);
};


#endif //MV_SLAM_SLAM_H
