//
// Created by xushen on 10/18/17.
//

#include "MapPaint.h"
#include <pangolin/pangolin.h>
using namespace std;

MapPaint::MapPaint() {
}

void MapPaint::drawPoint(const MapPoint &p) {
    glPointSize(1.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0,1.0,1.0);
    glVertex3f(p.x,p.y,p.z);
    glEnd();
}

void MapPaint::drawCamera(const KeyFrame &k) {
    //把下面的点都做一次旋转变换
    float Twc[16];
    for(int c = 0; c < 3; c++){
        for(int r = 0; r < 3; r++){
            Twc[r * 4 + c] = (float)k.rmat.at<double>(r,c);
        }
    }
    Twc[12] = (float)k.tvec.at<double>(0);
    Twc[13] = (float)k.tvec.at<double>(1);
    Twc[14] = (float)k.tvec.at<double>(2);
    Twc[3] = 0; Twc[7] = 0; Twc[11] = 0; Twc[15] = 1;
    glPushMatrix();
    //col major
    glMultMatrixf(Twc);

    //直线的创建
    const float w = 2;
    const float h = w*0.75;
    const float z = w*0.6;
    glLineWidth(2);
    glColor3f(1.0,0,0);
    glBegin(GL_LINES);

    glVertex3f(0,0,0);
    glVertex3f(w,h,z);
    glVertex3f(0,0,0);
    glVertex3f(w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,-h,z);
    glVertex3f(0,0,0);
    glVertex3f(-w,h,z);
    glVertex3f(w,h,z);
    glVertex3f(-w,h,z);
    glVertex3f(-w,h,z);
    glVertex3f(-w,-h,z);
    glVertex3f(-w,-h,z);
    glVertex3f(w,-h,z);
    glVertex3f(w,-h,z);
    glVertex3f(w,h,z);
    glEnd();

    glPopMatrix();
}

void MapPaint::drawMap(const vector<KeyFrame *> &allKeyFrames, const vector<MapPoint *> &pointClouds) {
    for(const KeyFrame * k: allKeyFrames){
        drawCamera(*k);
    }
    for(const MapPoint * p: pointClouds){
        drawPoint(*p);
    }
    // Swap frames and Process Events
}


