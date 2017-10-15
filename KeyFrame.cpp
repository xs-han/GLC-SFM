//
// Created by xushen on 10/14/17.
//

#include "KeyFrame.h"
#include <iostream>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/imgproc.hpp>
using namespace std;
using namespace cv;

Mat KeyFrame::cameraMartix;
cv::Ptr<cv::Feature2D> KeyFrame::detector;
FeatureMatcher KeyFrame::matcher;

bool KeyFrame::isFrameKey(const Mat &newFrame) {
    Mat newDesc;
    vector<KeyPoint> newKps;
    vector<DMatch> matches;
    matches.clear();
    detector->detectAndCompute(newFrame, noArray(), newKps, newDesc );
    matcher.match(kps, desc, newKps, newDesc, matches);
    return matches.size() < kps.size() * 0.6;
}

const Mat &KeyFrame::getRvec() const {
    return rvec;
}

void KeyFrame::setRvec(const Mat &rvec) {
    KeyFrame::rvec = rvec.clone();
}

const Mat &KeyFrame::getTvec() const {
    return tvec;
}

void KeyFrame::setTvec(const Mat &tvec) {
    KeyFrame::tvec = tvec.clone();
}

KeyFrame::KeyFrame(Mat newImg) {
    img = newImg.clone();
    detector->detectAndCompute(img, noArray(), kps, desc);
    rvec.create(3,1,CV_32F);
    tvec.create(3,1,CV_32F);
}
