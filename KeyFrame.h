//
// Created by xushen on 10/13/17.
//

#ifndef MV_SLAM_KEYFRAME_H
#define MV_SLAM_KEYFRAME_H

#include <iostream>
#include <opencv2/core.hpp>
#include "MapPoint.h"
#include "FeatureMatcher.h"
using namespace std;
using namespace cv;

class MapPoint;

class KeyFrame {
public:
    static Mat cameraMartix;
    static cv::Ptr<cv::Feature2D> detector;
    static FeatureMatcher matcher;
    Mat img;
    Mat rvec;
    Mat tvec;

    vector<KeyPoint> kps;
    vector<MapPoint* > mps;

    KeyFrame(Mat newImg);

    KeyFrame(KeyFrame & k) = default;

    bool isFrameKey(const Mat & newFrame);

    void computeRT(const vector<KeyFrame> & refKf);

    void detectKps();

    bool isFrameKey(const Mat &newFrame, vector<DMatch> &matches);
};


#endif //MV_SLAM_KEYFRAME_H
