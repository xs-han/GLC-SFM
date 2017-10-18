//
// Created by xushen on 10/14/17.
//

#include "KeyFrame.h"
#include <opencv2/highgui.hpp>
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
    Mat outimg;
    drawMatches(img, kps, newFrame, newKps, matches, outimg);
    namedWindow("matches");
    imshow("matches", outimg);
    cvWaitKey(0);
    destroyWindow("matches");
    return matches.size() < kps.size() * 1;
}

const Mat &KeyFrame::getRmat() const {
    return rmat;
}

void KeyFrame::setRmat(const Mat &rmat) {
    KeyFrame::rmat = rmat.clone();
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
    rmat = Mat::eye(3,3,CV_64F);
    tvec = Mat::zeros(3,1,CV_64F);
}

KeyFrame::KeyFrame(const Mat& newImg, Mat &R, Mat &t) {
    img = newImg.clone();
    detector->detectAndCompute(img, noArray(), kps, desc);
    mps.clear();
    mps.resize(kps.size(), nullptr);
    if (R.rows == 3 && 3 == R.cols){
        rmat = R.clone();
    } else if(R.rows == 1 || R.cols == 1){
        rmat.create(3,3,CV_64F);
        Rodrigues(R.clone(), rmat);
    } else{
        cout << "Invalid R data." << endl;
        exit(-2);
    }
    tvec = t.clone();
}

void KeyFrame::triangulateNewKeyFrame(const KeyFrame &newFrame,
                                      const vector<DMatch> & matches,
                                      Mat & res) {
    Mat R1, t1, projMatx1, R2, t2, projMatx2;
    vector<Point2f> points1(matches.size());
    vector<Point2f> points2(matches.size());

    R1 = rmat.clone();
    t1 = tvec.clone();
    hconcat(R1,t1,projMatx1);
    projMatx1 = KeyFrame::cameraMatrix * projMatx1.clone();

    R2 = newFrame.rmat.clone();
    t2 = newFrame.tvec.clone();
    hconcat(R2,t2,projMatx2);
    projMatx2 = KeyFrame::cameraMatrix * projMatx2.clone();

    int i = 0;
    for(const DMatch & m: matches){
        points1[i] = kps[m.queryIdx].pt;
        points2[i] = newFrame.kps[m.trainIdx].pt;
        i++;
    }

    triangulatePoints(projMatx1, projMatx2, points1, points2, res);
}


