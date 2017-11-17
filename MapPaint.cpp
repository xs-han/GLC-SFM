//
// Created by xushen on 10/18/17.
//

#include "MapPaint.h"
#include <pangolin/pangolin.h>
using namespace std;

MapPaint::MapPaint() {
}

void MapPaint::drawPoint(const MapPoint &p) {
    glPointSize(2.0f);
    glBegin(GL_POINTS);
    if(p.colored) {
        glColor3f(p.color[0] / (float)255.0, p.color[1] / (float)255.0, p.color[2] / (float)255.0);
    } else {
        glColor3f(1.0, 1.0, 1.0);
    }
    glVertex3f(p.x,p.y,p.z);
    glEnd();
}

void MapPaint::drawCamera(const KeyFrame &k) {
    //把下面的点都做一次旋转变换
    float homodata[] = {0,0,0,1};
    Mat Twc(4,4,CV_32F), homo(1,4,CV_32F,homodata);
    k.rmat.convertTo(Twc(Range(0,3), Range(0,3)),CV_32F);
    k.tvec.convertTo(Twc(Range(0,3), Range(3,4)),CV_32F);
    homo.convertTo(Twc(Range(3,4), Range(0,4)),CV_32F);
    Mat TwcT = Twc.inv().t();

    glPushMatrix();
    //col major
    glMultMatrixf((float*) TwcT.data);

    //直线的创建
    const float w = 0.5;
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

void MapPaint::drawCameraPair(KeyFrame & k1, KeyFrame & k2) {
    const Mat loc1 = k1.get3DLocation();
    const Mat loc2 = k2.get3DLocation();
    glLineWidth(4);
    glColor3f(0,0,1.0);
    glBegin(GL_LINES);
    glVertex3f(loc1.at<double>(0),loc1.at<double>(1),loc1.at<double>(2));
    glVertex3f(loc2.at<double>(0),loc2.at<double>(1),loc2.at<double>(2));
    glEnd();
}


void MapPaint::drawMap(const vector<KeyFrame *> &allKeyFrames,
                       const vector<vector<KeyFrame *> >&allVirtualFrames,
                       const vector<MapPoint *> &pointClouds,
                       const vector<pair<int,int>> loopPairs) {
    if(pointClouds.back()->colored)
        glClearColor(1.0f,1.0f,1.0f,1.0f);
    for(const KeyFrame * k: allKeyFrames){
        drawCamera(*k);
    }
    cout << allKeyFrames.back()->kfId << ": " << allKeyFrames.back()->computeReprojectionError() << " ";
    cout << endl;
    for(const vector<KeyFrame * > kvec: allVirtualFrames){
        for(const KeyFrame * k : kvec) {
            drawCamera(*k);
        }
    }
    for(const MapPoint * p: pointClouds){
        if(p->good && p->kfs.size() > 2)
            drawPoint(*p);
    }
    for(const pair<int, int> lp : loopPairs){
        drawCameraPair(*allKeyFrames[lp.first], *allKeyFrames[lp.second]);
    }

}


