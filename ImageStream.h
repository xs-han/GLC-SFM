//
// Created by xushen on 10/13/17.
//

#ifndef VIRTUALVIEW_IMAGESTREAM_H
#define VIRTUALVIEW_IMAGESTREAM_H

#include "MediaStream.h"
#include "dirent.h"
#include <iostream>
using namespace cv;
using namespace std;

class ImageStream: public MediaStream{
    //string inputPath;
    vector<string> allImages;
    vector<string>::iterator currentImagePtr;
    //bool finish;
public:
    bool isFinish() const;

public:
    ImageStream();

    explicit ImageStream(const string & folder);

    explicit ImageStream(const ImageStream & ims);

    void setInput(const string &folder);

    bool read(Mat &m);

    virtual ImageStream& operator >> (Mat & m);

    virtual explicit operator bool();

};

#endif //VIRTUALVIEW_IMAGESTREAM_H
