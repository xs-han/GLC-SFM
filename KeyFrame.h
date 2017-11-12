//
// Created by xushen on 10/13/17.
//

#ifndef MV_SLAM_KEYFRAME_H
#define MV_SLAM_KEYFRAME_H

#include <iostream>
#include <opencv2/core/core.hpp>
#include "MapPoint.h"
#include "FeatureMatcher.h"
using namespace std;
using namespace cv;

class MapPoint;

class KeyFrame {
public:
    static Mat cameraMatrix;
    static cv::Ptr<cv::Feature2D> detector;
    static FeatureMatcher matcher;
    static int nKeyFrames;
    Mat img;
    Mat rmat;
    Mat tvec;

    int kfId;

    vector<KeyPoint> kps;
    vector<MapPoint* > mps;
    Mat desc;
    vector<DMatch> mch;
    vector<MapPoint *> visibileMps;
    vector<KeyPoint> visibileKps;

    void setMch(const vector<DMatch> &mch);

    explicit KeyFrame(const Mat & newImg);

    KeyFrame(const Mat& newImg, Mat & R, Mat & t);

    KeyFrame(KeyFrame & k) = default;

    KeyFrame(const Mat & newImg, const vector<KeyPoint> & newKps, const Mat & newDesc);

    void computeNewKfRT(KeyFrame & newKf, const vector<DMatch> &mch, vector<int> & inliers);

    void setRmat(const Mat &rmat);

    void setTvec(const Mat &tvec);

    bool isFrameKey(const Mat &newFrame, vector<KeyPoint> &newKps, Mat &newDesc, vector<DMatch> &matches);

    double computeReprojectionError()const;

    void triangulateNewKeyFrame(const KeyFrame &newFrame, const vector<DMatch> &matches, vector<Point3f> &res,
                                vector<int> & isGood);

    void drawFrameMatches(const KeyFrame & f1, const KeyFrame & f2, const DMatch & m);

    void drawFrameMatches(const KeyFrame & f1, const KeyFrame & f2, const vector<DMatch> & m);

    Mat get3DLocation();

    void generateRt(KeyFrame *newkf, Mat relaRvec, Mat relaTvec);

    void generateVisibleMapPoints(vector<KeyFrame *> refKf);

    void generateImg(vector<KeyFrame *> refKf);
};


#endif //MV_SLAM_KEYFRAME_H
