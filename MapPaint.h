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

    void drawCamera(const KeyFrame & k);

    void drawPoint(const MapPoint & p);

    void drawCameraPair(KeyFrame &k1, KeyFrame &k2);

    void drawMap(const vector<KeyFrame *> &allKeyFrames,
                 const vector<vector<KeyFrame *>> &allVirtualFrames,
                 const vector<MapPoint *> &pointClouds,
                 const vector<pair<int, int>> loopPairs);
};


#endif //MV_SLAM_MAPPAINT_H
