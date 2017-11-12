//
// Created by xushen on 10/14/17.
//

#include "KeyFrame.h"
#include <opencv2/highgui.hpp>
#include <opencv2/sfm.hpp>
using namespace std;
using namespace cv;

Mat KeyFrame::cameraMatrix;
cv::Ptr<cv::Feature2D> KeyFrame::detector;
FeatureMatcher KeyFrame::matcher;
int KeyFrame::nKeyFrames = 0;

bool KeyFrame::isFrameKey(const Mat &newFrame, vector<KeyPoint> &newKps, Mat & newDesc, vector<DMatch> & matches) {
    newKps.clear();
    matches.clear();
    detector->detectAndCompute(newFrame, noArray(), newKps, newDesc );
    matcher.match(kps, desc, newKps, newDesc, matches);
    cout << matches.size() << endl;
    return matches.size() < kps.size() * 0.1;
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
    kfId = nKeyFrames;
    nKeyFrames++;
}

KeyFrame::KeyFrame(const Mat& newImg, Mat &R, Mat &t) {
    img = newImg.clone();
    detector->detectAndCompute(img, noArray(), kps, desc);
    mps.clear();
    mps.resize(kps.size(), nullptr);
    setRmat(R);
    setTvec(t);
    kfId = nKeyFrames;
    nKeyFrames++;
}

KeyFrame::KeyFrame(const Mat &newImg, const vector<KeyPoint> &newKps, const Mat &newDesc) {
    img = newImg.clone();
    kps = newKps;
    desc = newDesc.clone();
    mps.clear();
    mps.resize(kps.size(), nullptr);
    rmat = Mat::eye(3,3,CV_64F);
    tvec = Mat::zeros(3,1,CV_64F);
    kfId = nKeyFrames;
    nKeyFrames++;
}

void KeyFrame::triangulateNewKeyFrame(const KeyFrame &newFrame,
                                      const vector<DMatch> & matches,
                                      vector<Point3f> & res,
                                      vector <int> & isGood) {
    res.clear(); isGood.clear();

    Mat projMatx1(3,4,CV_64F), projMatx2(3,4,CV_64F);
    Mat k(3,3,CV_64F);
    vector<Point2f> points1(matches.size());
    vector<Point2f> points2(matches.size());

    KeyFrame::cameraMatrix.convertTo(k,CV_64F);
    rmat.convertTo(projMatx1(Range(0,3), Range(0,3)), CV_64F);
    tvec.convertTo(projMatx1.col(3), CV_64F);
    projMatx1 = k * projMatx1;

    newFrame.rmat.convertTo(projMatx2(Range(0,3), Range(0,3)), CV_64F);
    newFrame.tvec.convertTo(projMatx2.col(3), CV_64F);
    projMatx2 = k * projMatx2;

    int i = 0;
    for(const DMatch & m: matches){
        points1[i] = kps[m.queryIdx].pt;
        points2[i] = newFrame.kps[m.trainIdx].pt;
        i++;
    }

    isGood.resize(points1.size(), 1);
    int nGood = points1.size();
    cout << "matches: " << points1.size() << endl;
//    Mat resMat4d;
//    triangulatePoints(projMatx1, projMatx2, points1, points2, resMat4d);
//    assert(resMat4d.type() == CV_32F);
//
//    for(i = 0; i < resMat4d.cols; i++) {
//        Mat p = resMat4d.col(i);
//        Point3f pp((p.at<float>(0) / p.at<float>(3)),
//                   (p.at<float>(1) / p.at<float>(3)),
//                   (p.at<float>(2) / p.at<float>(3)));
//        res.push_back(pp);
//    }

    std::vector<Mat > point2d;
    point2d.push_back(Mat(points1).reshape(1).t()); point2d.push_back(Mat(points2).reshape(1).t());
    vector<Mat> projection_matrices;
    projection_matrices.push_back(projMatx1);projection_matrices.push_back(projMatx2);

    Mat point3d;
    sfm::triangulatePoints(point2d, projection_matrices, point3d);
    assert(point3d.type()==CV_64F);

    vector<DMatch> badmatches;
    for(i = 0; i < point3d.cols; i++) {
        Mat p = point3d.col(i);
        Point3f pp((float)(p.at<double>(0)),
                   (float)(p.at<double>(1)),
                   (float)(p.at<double>(2)));
        res.push_back(pp);

        Mat pp1local = rmat * p + tvec;
        Mat pp2local = newFrame.rmat * p + newFrame.tvec;

        if(pp1local.at<double>(2) < 0 || pp2local.at<double>(2) < 0){
            isGood[i] = 0;
            nGood--;
            badmatches.push_back(matches[i]);
        }
    }
    cout << "good trianglation: " << nGood << endl;
    //drawFrameMatches(*this, newFrame, badmatches);


    Mat rvec1, rvec2; vector<Point2f> imagePoints1, imagePoints2;
    Rodrigues(rmat, rvec1); Rodrigues(newFrame.rmat, rvec2);
    projectPoints( res, rvec1, tvec, cameraMatrix, noArray(), imagePoints1);
    projectPoints( res, rvec2, newFrame.tvec, cameraMatrix, noArray(), imagePoints2);
    Mat c1 = -1 * rmat.inv() * tvec;
    Mat c2 = -1 * newFrame.rmat.inv() * newFrame.tvec;
    for(i = 0; i < res.size(); i++){
        if(isGood[i]) {
            if (norm(Mat(points1[i]), Mat(imagePoints1[i])) > 5 ||
                norm(Mat(points2[i]), Mat(imagePoints2[i])) > 5) {
                isGood[i] = 0;
                nGood--;
                badmatches.push_back(matches[i]);
                continue;
            } else {
                isGood[i] = 1;
            }

            Mat p(3, 1, CV_64F, {res[i].x, res[i].y, res[i].z});
            Mat p1 = c1 - p, p2 = c2 - p;
            double cosAngle = norm(p1.t() * p2) / (norm(p1) * norm(p2));
            assert(cosAngle <= 1.1);
//            assert(cosAngle >= -1);
//            if (cosAngle > 1) {
//                isGood[i] = 0;
//                nGood--;
//                badmatches.push_back(matches[i]);
//            }
        }
    }

//    vector<DMatch> goodmatches;
//    for(int i = 0; i < isGood.size(); i++){
//        if(isGood[i]){
//            goodmatches.push_back(matches[i]);
//        }
//    }
//    drawFrameMatches(*this, newFrame, goodmatches);
    cout << "successful trianglation: " << nGood << endl;
}

void KeyFrame::setMch(const vector<DMatch> &newMatches) {
    mch = newMatches;
}

void KeyFrame::computeNewKfRT(KeyFrame &newKf, const vector<DMatch> &mch, vector<int> & inliers) {
    vector<Point3f> obPoints;
    vector<Point2f> imgPoints;
    Mat rvec, tvec;
    for(const DMatch & m : mch){
        MapPoint * p = mps[m.queryIdx];
        if(p != nullptr && p->good) {
            obPoints.emplace_back(p->x, p->y, p->z);
            imgPoints.push_back(newKf.kps[m.trainIdx].pt);
        }
    }
    cout << "useful match: " << obPoints.size() << endl;
    solvePnPRansac(obPoints, imgPoints, cameraMatrix, noArray(), rvec, tvec, false, 100, 1.0, 0.99, inliers, CV_ITERATIVE);

    newKf.setRmat(rvec);
    newKf.setTvec(tvec);
}

double KeyFrame::computeReprojectionError()const {
    vector<Point3f> objectPoints;
    vector<Point2f> imagePoints;
    vector<Point2f> imagePoints2;
    Mat rvec(3,1,CV_64F);
    Rodrigues(rmat, rvec);
    for(int i = 0; i < mps.size(); i++){
        if(mps[i] != nullptr && mps[i]->good){
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


void KeyFrame::drawFrameMatches(const KeyFrame & f1, const KeyFrame & f2, const DMatch & m){
    vector<DMatch> vm;
    vm.push_back(m);
    Mat out;
    drawMatches(f1.img, f1.kps, f2.img, f2.kps, vm, out);
    namedWindow("single match", 0);
    imshow("single match", out);
    cvWaitKey(0);
    destroyWindow("single match");
}

void KeyFrame::drawFrameMatches(const KeyFrame & f1, const KeyFrame & f2, const vector<DMatch> & m){
    Mat out;
    drawMatches(f1.img, f1.kps, f2.img, f2.kps, m, out);
    namedWindow("multiple matches", 0);
    imshow("multiple matches", out);
    cvWaitKey(0);
    destroyWindow("multiple matches");
}

Mat KeyFrame::get3DLocation() {
    return -rmat.inv() * tvec;
}

void KeyFrame::generateRt(KeyFrame *oldkf, Mat relaRvec, Mat relaTvec) {
    Mat relaRmat;
    Rodrigues(relaRvec, relaRmat);
    setRmat(relaRmat*oldkf->rmat);
    setTvec(relaRmat*(oldkf->tvec) + relaTvec);
}

void KeyFrame::generateVisibleMapPoints(vector<KeyFrame * > refKf){
    for(KeyFrame * kf : refKf){
        for(MapPoint * p : kf->mps){
            if(p != nullptr && p->good) {
                Mat p3d(3,1,CV_64F);
                p3d.at<double>(0) = p->x;p3d.at<double>(1) = p->y;p3d.at<double>(2) = p->z;
                Mat local3DPoint = rmat * p3d + tvec;
                if (local3DPoint.at<double>(2) > 0) {
                    Mat homo2DPoint = cameraMatrix * local3DPoint;
                    Point3f l3d(homo2DPoint.at<double>(0),
                                homo2DPoint.at<double>(1),
                                homo2DPoint.at<double>(2));
                    if (l3d.x / l3d.z < kf->img.cols && l3d.x / l3d.z > 0
                        && l3d.y / l3d.z < kf->img.rows && l3d.y / l3d.z > 0) {
                        if (find(visibileMps.begin(), visibileMps.end(), p) == visibileMps.end()) {
                            visibileMps.push_back(p);
                            visibileKps.push_back(KeyPoint(l3d.x / l3d.z, l3d.y / l3d.z, 0));
                        }
                    }
                }
            }
        }
    }
}

void KeyFrame::generateImg(vector<KeyFrame *> refKf){
    Mat newImg(refKf.back()->img.rows, refKf.back()->img.cols, CV_8UC3);
    assert(visibileKps.size() == visibileMps.size());
    Mat mask(refKf.back()->img.rows, refKf.back()->img.cols, CV_8U);
    for(int i = 0; i < newImg.rows; i++){
        for(int j = 0; j < newImg.cols; j++){
            Vec3b p; p[0] = 0;p[1] = 0;p[2] = 0;
            newImg.at<Vec3b>(i,j) = p;
            mask.at<uchar>(i,j) = 0;
        }
    }

    int l = 30;
    for(int i = 0; i < visibileKps.size(); i++){
        MapPoint * refMp = visibileMps[i];
        Mat refMpmat(3,1,CV_64F);
        refMpmat.at<double>(0) = refMp->x;refMpmat.at<double>(1) = refMp->y;refMpmat.at<double>(2) = refMp->z;
        Mat localRefMp = rmat * refMpmat + tvec;
        int u = (int)visibileKps[i].pt.x;
        int v = (int)visibileKps[i].pt.y;
        Mat homoP2d(3,1,CV_64F);
        Mat rmatinv = rmat.inv();
        Mat cminv = cameraMatrix.inv();
        for(int r = max(0,v - l); r < min(v + l, newImg.rows); r++){
            for(int c = max(0,u - l); c < min(u + l, newImg.cols); c++){
                if(mask.at<uchar>(r,c) != 0){
                    continue;
                }
                homoP2d.at<double>(0) = c; homoP2d.at<double>(1) = r; homoP2d.at<double>(2) = 1;
                Mat p3d = rmatinv * (cminv *(localRefMp.at<double>(2) * homoP2d) - tvec);
                int numOp = 0;
                Vec3i obv;obv[0] = 0;obv[1] = 0;obv[2] = 0;
                for(KeyFrame * kf : refKf ){
                    Mat homoKfP2d = cameraMatrix * (kf->rmat * p3d + kf->tvec);
                    assert(homoKfP2d.type() == CV_64F);
                    int kfR = homoKfP2d.at<double>(1) / homoKfP2d.at<double>(2);
                    int kfC = homoKfP2d.at<double>(0) / homoKfP2d.at<double>(2);
                    if(kfR >= 0 && kfR < kf->img.rows && kfC >= 0 && kfC < kf->img.cols){
                        numOp += 1;
                        obv += kf->img.at<Vec3b>(kfR, kfC);
                        break;
                    }
                }
                if(numOp > 0) {
                    mask.at<uchar>(r, c) = 1;
                    newImg.at<Vec3b>(r,c) = obv / numOp;
                } else{
                    mask.at<uchar>(r, c) = 0;
                }
            }
        }
    }

    img = newImg.clone();
    Mat out;
    drawKeypoints(newImg,visibileKps, out);
    namedWindow("newGenImg", 0);
    imshow("newGenImg", out);
    cvWaitKey(0);
    destroyWindow("newGenImg");
}

