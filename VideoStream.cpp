//
// Created by xushen on 10/13/17.
//

#include "VideoStream.h"
#include <iostream>
#include <opencv/cv.hpp>

using namespace std;
using namespace cv;

VideoStream::VideoStream(const string &vName) :MediaStream(vName) //inputPath(vName),
                                                                        //, finish(false)
{
    setInput(vName);
    videoHeight = static_cast<int>(cap.get(CV_CAP_PROP_FRAME_HEIGHT));
    videoWidth = static_cast<int>(cap.get(CV_CAP_PROP_FRAME_WIDTH));
}

VideoStream::VideoStream()  : MediaStream("unknown"),
        //inputPath("unknown"),
                              videoWidth(-1), videoHeight(-1)
//,finish(false)
{}

VideoStream::VideoStream(const VideoStream & imVideo):MediaStream(imVideo),
        //inputPath(imVideo.inputPath),
                                                      videoWidth(imVideo.videoWidth),
                                                      videoHeight(imVideo.videoHeight), frameRate(imVideo.frameRate),
                                                      frame(imVideo.frame.clone()), cap(imVideo.cap)
//,finish(imVideo.finish)
{}

void VideoStream::showVideo(bool enableKeyStop) {
    Mat newFrame;
    namedWindow(inputPath,CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"
    while(read(newFrame)){
        imshow(inputPath, newFrame);
        int k = 0;
        if (enableKeyStop && (k = waitKey(30)) >= 0) {
            if(k == 'q'){
                break;
            }
            waitKey(0);
        }
    }
}

void VideoStream::setInput(const string &videoName) {
    inputPath = videoName;
    cap.open(videoName);
    if(!cap.isOpened()){
        cerr << "Open input video error." << endl;
        exit(-1);
    }  // check if we succeeded
    frameRate = cap.get(CV_CAP_PROP_FPS);
    id = 0;
}

VideoStream & VideoStream::operator>>(Mat &m) {
    read(m);
    return (*this);
}

bool VideoStream::read(Mat &m) {
    int loops = 0;
    while(!cap.read(frame)){
        loops += 1;
        if(loops == 200){
            break;
        }
    }
    if(loops == 200){
        cout  << "video is over." << endl;
        finish = true;
        return false;
    }
    else {
        id += 1;
        resize(frame, m, Size(videoWidth, videoHeight));
        if (loops > 0) {
            cerr << "warning: An empty frame is founded and discarded. " << endl;
            return true;
        }
        else {
            return true;
        }
    }
}

int VideoStream::getVideoWidth() const {
    return videoWidth;
}

void VideoStream::setVideoWidth(int videoWidth) {
    VideoStream::videoWidth = videoWidth;
}

int VideoStream::getVideoHeight() const {
    return videoHeight;
}

void VideoStream::setVideoHeight(int videoHeight) {
    VideoStream::videoHeight = videoHeight;
}

double VideoStream::getFrameRate() const {
    return frameRate;
}

VideoStream::operator bool() {
    return !finish;
}

bool VideoStream::isFinish() const {
    return finish;
}
