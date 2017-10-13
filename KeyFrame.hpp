//
// Created by xushen on 10/13/17.
//

#ifndef MV_SLAM_KEYFRAME_H
#define MV_SLAM_KEYFRAME_H

#include <iostream>
#include <opencv2/core.hpp>
#include "MapPoint.hpp"
using namespace std;
using namespace cv;

class MapPoint;

class KeyFrame {
public:

    static Mat cameraMartix;
    Mat img;
    Mat Rmat;
    Mat Tmat;

    vector<KeyPoint> kps;
    vector<MapPoint* > mps;

    KeyFrame(Mat & rvec, Mat tvac, Mat newImg);

    KeyFrame(KeyFrame & k) = default;

    void computeRT(const vector<KeyFrame> & refKf);

    void detectKps();

};


#endif //MV_SLAM_KEYFRAME_H
