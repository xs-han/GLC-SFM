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
    cout << matches.size() << endl;
    return matches.size() < kps.size() * 0.3;
}

void KeyFrame::setRmat(const Mat & R) {
    if (R.rows == 3 && 3 == R.cols){
        rmat = R.clone();
    } else if(R.rows == 1 || R.cols == 1){
        rmat.create(3,3,CV_64F);
        Rodrigues(R.clone(), rmat);
    } else{
        cout << "Invalid R data." << endl;
        exit(-2);
    }
}

void KeyFrame::setTvec(const Mat &t) {
    tvec = t.clone();
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
    setRmat(R);
    setTvec(t);
}

KeyFrame::KeyFrame(const Mat &newImg, const vector<KeyPoint> &newKps, const Mat &newDesc) {
    img = newImg.clone();
    kps = newKps;
    desc = newDesc.clone();
    mps.clear();
    mps.resize(kps.size(), nullptr);
    rmat = Mat::eye(3,3,CV_64F);
    tvec = Mat::zeros(3,1,CV_64F);
}

/**
 From "Triangulation", Hartley, R.I. and Sturm, P., Computer vision and image understanding, 1997
 */
void LinearLSTriangulation(const vector<Point2d> uvec,       //homogenous image point (u,v,1)
                           Mat P,       //camera 1 matrix
                           vector<Point2d> u1vec,      //homogenous image point in 2nd camera
                           Mat P1,       //camera 2 matrix
                           Mat & res
)
{
    //build matrix A for homogenous equation system Ax = 0
    //assume X = (x,y,z,1), for Linear-LS method
    //which turns it into a AX = B system, where A is 4x3, X is 3x1 and B is 4x1
    Mat tmp;
    int n = (int) uvec.size();
    for(int i = 0; i < n; i++) {
        const Point2d & u = uvec[i];
        const Point2d & u1 = u1vec[i];
        Matx43d A(u.x * P.at<double>(2, 0) - P.at<double>(0, 0), u.x * P.at<double>(2, 1) - P.at<double>(0, 1), u.x * P.at<double>(2, 2) - P.at<double>(0, 2),
                  u.y * P.at<double>(2, 0) - P.at<double>(1, 0), u.y * P.at<double>(2, 1) - P.at<double>(1, 1), u.y * P.at<double>(2, 2) - P.at<double>(1, 2),
                  u1.x * P1.at<double>(2, 0) - P1.at<double>(0, 0), u1.x * P1.at<double>(2, 1) - P1.at<double>(0, 1), u1.x * P1.at<double>(2, 2) - P1.at<double>(0, 2),
                  u1.y * P1.at<double>(2, 0) - P1.at<double>(1, 0), u1.y * P1.at<double>(2, 1) - P1.at<double>(1, 1), u1.y * P1.at<double>(2, 2) - P1.at<double>(1, 2)
        );
        Matx41d B(-(u.x * P.at<double>(2, 3) - P.at<double>(0, 3)),
                  -(u.y * P.at<double>(2, 3) - P.at<double>(1, 3)),
                  -(u1.x * P1.at<double>(2, 3) - P1.at<double>(0, 3)),
                  -(u1.y * P1.at<double>(2, 3) - P1.at<double>(1, 3)));

        Mat X;
        solve(A, B, X, DECOMP_SVD);
        tmp.push_back(X.t());
    }

    res = tmp.clone().t();
}

void KeyFrame::triangulateNewKeyFrame(const KeyFrame &newFrame,
                                      const vector<DMatch> & matches,
                                      vector<Point3f> & res,
                                      vector <int> & isGood) {
    Mat resMat4d;
    res.clear(); isGood.clear();

    Mat projMatx1(3,4,CV_32FC1), projMatx2(3,4,CV_32FC1);
    Mat k(3,3,CV_32FC1);
    vector<Point2f> points1(matches.size());
    vector<Point2f> points2(matches.size());

    KeyFrame::cameraMatrix.convertTo(k,CV_32FC1);
    rmat.convertTo(projMatx1(Range(0,3), Range(0,3)), CV_32FC1);
    tvec.convertTo(projMatx1.col(3), CV_32FC1);
    projMatx1 = k * projMatx1;

    newFrame.rmat.convertTo(projMatx2(Range(0,3), Range(0,3)), CV_32FC1);
    newFrame.tvec.convertTo(projMatx2.col(3), CV_32FC1);;
    projMatx2 = k * projMatx2;

    int i = 0;
    for(const DMatch & m: matches){
        points1[i] = kps[m.queryIdx].pt;
        points2[i] = newFrame.kps[m.trainIdx].pt;
        i++;
    }
    //LinearLSTriangulation(points1, projMatx1, points2, projMatx2, res);
    //cout << projMatx1 << endl;
    //cout << projMatx2 << endl;
    triangulatePoints(projMatx1, projMatx2, points1, points2, resMat4d);
    assert(resMat4d.type() == CV_32F);
    for(i = 0; i < resMat4d.cols; i++) {
        Mat p = resMat4d.col(i);
        Point3f pp((p.at<float>(0) / p.at<float>(3)),
                   (p.at<float>(1) / p.at<float>(3)),
                   (p.at<float>(2) / p.at<float>(3)));
        res.push_back(pp);
    }
    isGood.resize(res.size(), 1);

    Mat rvec1, rvec2; vector<Point2f> imagePoints1, imagePoints2;
    Rodrigues(rmat, rvec1); Rodrigues(newFrame.rmat, rvec2);
    projectPoints( res, rvec1, tvec, cameraMatrix, noArray(), imagePoints1);
    projectPoints( res, rvec2, newFrame.tvec, cameraMatrix, noArray(), imagePoints2);
    Mat c1 = -1 * rmat.inv() * tvec; c1.convertTo(c1, CV_32F);
    Mat c2 = -1 * newFrame.rmat.inv() * newFrame.tvec; c2.convertTo(c2, CV_32F);
    for(i = 0; i < res.size(); i++){
        if(norm(Mat(points1[i]), Mat(imagePoints1[i])) > 1 ||
                norm(Mat(points2[i]), Mat(imagePoints2[i])) > 1){
            isGood[i] = 0;
        }
        else{
            isGood[i] = 1;
        }
        Mat p(3,1,CV_32F, {res[i].x, res[i].y, res[i].z});
        Mat p1 = c1 - p, p2 = c2 - p;
        double cosAngle = norm(p1.t() * p2) / (norm(p1) * norm(p2));
        assert(cosAngle <= 1.1);
        assert(cosAngle >= -1);
        if(cosAngle > 0.9999 || cosAngle < 0){
            isGood[i] = 0;
        }
    }
}

void KeyFrame::setMch(const vector<DMatch> &newMatches) {
    mch = newMatches;
}

void KeyFrame::computeNewKfRT(KeyFrame &newKf, const vector<DMatch> &mch) {
    vector<Point3f> obPoints;
    vector<Point2f> imgPoints;
    Mat rvec, tvec;
    for(const DMatch & m : mch){
        MapPoint * p = mps[m.queryIdx];
        if(p != nullptr) {
            obPoints.emplace_back(p->x, p->y, p->z);
            imgPoints.push_back(newKf.kps[m.trainIdx].pt);
        }
    }
    vector<int> inlier;
    solvePnPRansac(obPoints, imgPoints, cameraMatrix, noArray(), rvec, tvec, false, 500, 1.0, 0.99, inlier, CV_ITERATIVE);
    //cout << inlier << endl;
    newKf.setRmat(rvec);
    newKf.setTvec(tvec);
}

double KeyFrame::computeReprojectionError()const {
    vector<Point3f> objectPoints;
    vector<Point2f> imagePoints;
    vector<Point2f> imagePoints2;
    Mat rvec(3,1,CV_32F);
    Rodrigues(rmat, rvec);
    for(int i = 0; i < mps.size(); i++){
        if(mps[i] != nullptr){
            imagePoints.emplace_back(kps[i].pt);
            objectPoints.emplace_back(mps[i]->x, mps[i]->y, mps[i]->z);
        }
    }
    double err = 0;
    projectPoints( Mat(objectPoints), rvec, tvec, cameraMatrix,
                       noArray(), imagePoints2);
    err = norm(Mat(imagePoints), Mat(imagePoints2), CV_L2) / sqrt((double)imagePoints.size());
    return err;
}

