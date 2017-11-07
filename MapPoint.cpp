//
// Created by xushen on 10/30/17.
//

#include "MapPoint.h"
#include <opencv2/sfm.hpp>

void MapPoint::deleteMapPoint() {
    for(int i = 0; i < kfs.size(); i++){
        kfs[i]->mps[kps[i]] = nullptr;
    }
}

bool MapPoint::correctMapPoint(KeyFrame *newKf, int kp) {
//    vector<Mat> point2d, projMats;
//    Mat point3d;
//    for(int i = 0; i < kps.size(); i++){
//        vector<Point2f> p(1);
//        p[0] = kfs[i]->kps[kps[i]].pt;
//        point2d.push_back(Mat(p).reshape(1).t());
//        Mat k = kfs[i]->cameraMatrix;
//        Mat Rt; hconcat(kfs[i]->rmat, kfs[i]->tvec, Rt);
//        projMats.push_back(k * Rt);
//    }
////    vector<Point2f> p(1);
////    p[0] = newKf->kps[kp].pt;
////    point2d.push_back(Mat(p).reshape(1).t());
////    Mat k = newKf->cameraMatrix;
////    Mat Rt; hconcat(newKf->rmat, newKf->tvec, Rt);
////    projMats.push_back(k * Rt);
//    sfm::triangulatePoints(point2d, projMats, point3d);
////    cout << "[ x, y, z ] = " << x << ", " << y << ", " << z << endl;
////    cout << "view size: " << kfs.size() << endl;
////    cout << point3d << endl;
//    x = (float)point3d.at<double>(0);
//    y = (float)point3d.at<double>(1);
//    z = (float)point3d.at<double>(2);
//    good = isGood(3) && isGood(newKf,kp, 3);
//    if(!good){
//        deleteMapPoint();
//    }
//    return good;
}

bool MapPoint::isGood(double err){
    vector<Point3f> p;p.emplace_back(x,y,z);
    int i = 0;
    for(const KeyFrame * f : kfs){
        Mat rmat = f->rmat.clone(), tvec = f->tvec.clone();
        Mat rvec; Rodrigues(rmat, rvec);
        vector<Point2f> imagePoints;
        projectPoints(p, rvec, tvec, f->cameraMatrix, noArray(), imagePoints);
        if (norm(Mat(f->kps[kps[i]].pt), Mat(imagePoints[0])) > err){
            return false;
        }
        i++;
    }
    return true;
}

bool MapPoint::isGood(KeyFrame * f, int kp, double err){
    vector<Point3f> p;p.emplace_back(x,y,z);
    Mat rmat = f->rmat.clone(), tvec = f->tvec.clone();
    Mat rvec; Rodrigues(rmat, rvec);
    vector<Point2f> imagePoints;
    projectPoints(p, rvec, tvec, f->cameraMatrix, noArray(), imagePoints);
    return norm(Mat(f->kps[kp].pt), Mat(imagePoints[0])) <= err;
}

void MapPoint::setColor(int r, int g, int b) {
    color[0] = r; color[0] = g; color[0] = b;
    colored = true;
}

void MapPoint::addColor(int r, int g, int b) {
    if(colored){
        color[0] = r;
        color[1] = g;
        color[2] = b;
    } else{
        setColor(r,g,b);
    }
}

void MapPoint::addKf(KeyFrame &f, int p) {
    kfs.push_back(&f); kps.push_back(p);
}

void MapPoint::addKf(KeyFrame &f, int p, bool coloredMap) {
    kfs.push_back(&f); kps.push_back(p);
    if(coloredMap){
        Vec3b col = f.img.at<Vec3b>(f.kps[p].pt.y, f.kps[p].pt.x);
        addColor(col.val[2], col.val[1], col.val[0]);
    }
}

