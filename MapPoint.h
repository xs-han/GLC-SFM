//
// Created by xushen on 10/13/17.
//

#ifndef MV_SLAM_MAPPOINT_H
#define MV_SLAM_MAPPOINT_H

#include <iostream>
#include <opencv2/core.hpp>
#include "KeyFrame.h"
using namespace std;
using namespace cv;

class KeyFrame;

class MapPoint {
public:
    float x, y, z;

    bool good;
    vector<KeyFrame *> kfs;
    vector<int> kps;

    explicit MapPoint(const Point3f & p):x(p.x), y(p.y), z(p.z), good(true){kfs.clear(); kps.clear();};
    MapPoint(float x1, float y1, float z1):x(x1), y(y1), z(z1), good(true){};
    void addKf(KeyFrame & f, int p){kfs.push_back(&f); kps.push_back(p);}
    void deleteMapPoint();

    friend ostream &operator<<(ostream &os, const MapPoint &point) {
        os << "x: " << point.x << " y: " << point.y << " z: " << point.z;
        return os;
    }

    bool isGood(double err);

    bool isGood(KeyFrame *f, int kp, double err);

    bool correctMapPoint(KeyFrame *newKf, int kp);
};


#endif //MV_SLAM_MAPPOINT_H
