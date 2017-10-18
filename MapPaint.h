//
// Created by xushen on 10/18/17.
//

#ifndef MV_SLAM_MAPPAINT_H
#define MV_SLAM_MAPPAINT_H

#include "MapPaint.h"
#include "KeyFrame.h"
#include <pangolin/pangolin.h>

class MapPaint {
public:

    MapPaint();

    void drawMap(const vector<KeyFrame *> & allKeyFrames, const vector<MapPoint *> & pointClouds);

    void drawCamera(const KeyFrame & k);

    void drawPoint(const MapPoint & p);
};


#endif //MV_SLAM_MAPPAINT_H
