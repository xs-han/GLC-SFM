//
// Created by xushen on 9/11/17.
//

#ifndef DESCSINMUTIVIEWS_IMPORTVIDEO_H
#define DESCSINMUTIVIEWS_IMPORTVIDEO_H

#include "MediaStream.h"
#include <opencv2/highgui.hpp>

#define Debug
using namespace std;
using namespace cv;

class VideoStream: public MediaStream{
protected:
    //string inputPath;
    int videoWidth;
    int videoHeight;
    double frameRate;
    Mat frame;
    VideoCapture cap;
    //bool finish;

public:
    explicit VideoStream(const string &vName);

    VideoStream();

    VideoStream(const VideoStream &);

    void setInput(const string &videoName) override;

    bool read(Mat & m) override;

    virtual VideoStream& operator >> (Mat & m);

    virtual explicit operator bool() ;

    void showVideo(bool enableKeyStop);

    int getVideoWidth() const;

    void setVideoWidth(int videoWidth);

    int getVideoHeight() const;

    void setVideoHeight(int videoHeight);

    double getFrameRate() const;



};

#endif //DESCSINMUTIVIEWS_IMPORTVIDEO_H
