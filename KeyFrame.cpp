//
// Created by xushen on 10/14/17.
//

#include "KeyFrame.h"
using namespace std;
using namespace cv;

Mat KeyFrame::cameraMatrix;
cv::Ptr<cv::Feature2D> KeyFrame::detector;
FeatureMatcher KeyFrame::matcher;

bool KeyFrame::isFrameKey(const Mat &newFrame, vector<KeyPoint> &newKps, Mat & newDesc, vector<DMatch> & matches) {
    newKps.clear();
    matches.clear();
    detector->detectAndCompute(newFrame, noArray(), newKps, newDesc );
    matcher.match(kps, desc, newKps, newDesc, matches);
    return matches.size() < kps.size() * 1;
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

KeyFrame::KeyFrame(const Mat &newImg) {
    img = newImg.clone();
    detector->detectAndCompute(img, noArray(), kps, desc);
    mps.clear();
    mps.resize(kps.size(), nullptr);
    rvec.create(3,1,CV_32F);
    tvec.create(3,1,CV_32F);
}

KeyFrame::KeyFrame(const Mat& newImg, Mat &R, Mat &t) {
    img = newImg.clone();
    detector->detectAndCompute(img, noArray(), kps, desc);
    mps.clear();
    mps.resize(kps.size(), nullptr);
    if (R.rows == 3 && 3 == R.cols){
        rvec.create(3,1,CV_32F);
        Rodrigues(R.clone(), rvec);
    } else{
        rvec = R.clone();
    }
    tvec = t.clone();
}

void KeyFrame::triangulateNewKeyFrame(const KeyFrame &newFrame,
                                      const vector<DMatch> & matches,
                                      vector<Point2f> & res) {
    Mat R1, t1, projMatx1, R2, t2, projMatx2;
    vector<Point2f> points1(matches.size());
    vector<Point2f> points2(matches.size());

    R1.create(3,3,CV_32F);
    Rodrigues(rvec, R1);
    t1 = tvec.clone();
    hconcat(R1,t1,projMatx1);

    R2.create(3,3,CV_32F);
    Rodrigues(newFrame.rvec, R2);
    t2 = newFrame.tvec.clone();
    hconcat(R2,t2,projMatx2);

    int i = 0;
    for(const DMatch & m: matches){
        points1[i] = kps[m.queryIdx].pt;
        points2[i] = newFrame.kps[m.trainIdx].pt;
        i++;
    }

    triangulatePoints(projMatx1, projMatx2, points1, points2, res);
}


