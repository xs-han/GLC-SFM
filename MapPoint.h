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
    vector<KeyFrame *> kfs;
    vector<KeyPoint *> kps;

    explicit MapPoint(const Point3f & p):x(p.x), y(p.y), z(p.z){kfs.clear(); kps.clear();};
    MapPoint(float x1, float y1, float z1):x(x1), y(y1), z(z1){};
    void addKf(KeyFrame & f, KeyPoint & p){kfs.push_back(&f); kps.push_back(&p);}

    friend ostream &operator<<(ostream &os, const MapPoint &point) {
        os << "x: " << point.x << " y: " << point.y << " z: " << point.z;
        return os;
    }
};


#endif //MV_SLAM_MAPPOINT_H
