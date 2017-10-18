#include <iostream>
#include <pangolin/pangolin.h>

using namespace std;

void drawCamera(const float * Twc){
    //把下面的点都做一次旋转变换
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

void drawPoint(float x, float y, float z){
    //点的创建
    glPointSize(10.0f);
    glBegin(GL_POINTS);
    glColor3f(1.0,1.0,1.0);
    glVertex3f(x,y,z);
    glEnd();
}

int main(int argc, char **argv)
{

    while(!pangolin::ShouldQuit())
    {
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
        pangolin::glDrawColouredCube();\
        //坐标轴的创建
        pangolin::glDrawAxis(3);

        z += 0.01;
        drawPoint(x,y,z);
        Twc[14] += 0.01;
        allCams.push_back(Twc);
        for(vector<float> tmp: allCams){
            drawCamera(tmp.data());
        }

        // Swap frames and Process Events
        pangolin::FinishFrame();
    }

    return 0;

}