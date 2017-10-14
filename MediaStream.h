//
// Created by xushen on 10/13/17.
//

#ifndef MV_SLAM_MEDIASTREAM_H
#define MV_SLAM_MEDIASTREAM_H

#include <string>
#include <opencv2/core.hpp>
using namespace std;
using namespace cv;

class MediaStream{
public:
    string inputPath;
    bool finish;

    virtual bool isFinish() const {
        return finish;
    }

    MediaStream():inputPath("unknown"), finish(true){};

    explicit MediaStream(const string & name): inputPath(name), finish(false){};

    virtual void setInput(const string & name) = 0;

    virtual bool read(Mat & m) = 0;

    virtual MediaStream& operator >> (Mat & m) = 0;

    virtual explicit operator bool() = 0;


};


#endif //MV_SLAM_MEDIASTREAM_H
