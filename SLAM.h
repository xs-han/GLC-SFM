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
#include "DBoW3.h"

using namespace cv;
using namespace std;
using namespace DBoW3;

class SLAM {
public:
    bool rectified;
    int imageScale;
    Mat cameraMatrix;
    Mat m1, m2, newCameraMatrix;
    cv::Ptr<cv::Feature2D> KpsDetector;
    cv::Ptr<cv::Feature2D> DescDetector;
    FeatureMatcher matcher;

    Mat distortionCoefficient;
    vector<MapPoint *> pointClouds;
    vector<KeyFrame *> allKeyFrames;

    vector<vector<KeyFrame *> > allVirtualFrames;
    DBoW3::Vocabulary voc;
    DBoW3::Database db;
    vector<pair<int,int> > loopPair;

    MediaStream * ms;
    MapPaint mPaint;
    bool coloredMap;
    ofstream outRes;

    explicit SLAM(string settingFile = "../cfg/setting.xml");

    void setCameraIntrinsicParams(string calibFile);

    void process();

    void initialize();

    void undistortFrame(Mat &input, Mat &output);

    void track(KeyFrame & k, const vector <DMatch> & matches);

    bool localmap(KeyFrame & k, const vector <DMatch> & matches);

    bool loopclose(int delay, int refSize, int threhold);

    //void generateVirtualFrames();

    bool loopcloseReal(int delay);
};


#endif //MV_SLAM_SLAM_H
