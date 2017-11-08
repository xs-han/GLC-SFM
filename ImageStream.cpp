//
// Created by xushen on 10/13/17.
//

#include "ImageStream.h"
#include <opencv2/highgui.hpp>

ImageStream::ImageStream(const string &folder):MediaStream(folder) //,inputPath(folder)
{
    allImages.clear();
    setInput(folder);
}

ImageStream::ImageStream() {
    allImages.clear();
    inputPath = "unknown";
}

void ImageStream::setInput(const string &folder) {
    allImages.clear();
    if (*(inputPath.end() - 1) != '/') {
        inputPath += "/";
    }
    // get all file names
    DIR * pdir;
    struct dirent *entry;
    if ((pdir = opendir(inputPath.c_str()))) {
        while ((entry = readdir(pdir))) {
            if (strcmp(entry->d_name, ".") != 0
                && strcmp(entry->d_name, "..") != 0
                && (strstr(entry->d_name, ".png") || strstr(entry->d_name, ".jpg"))) {
                allImages.push_back(inputPath + entry->d_name);
                //cout << "found file:" << *(allImages.end()-1) << endl;
            }
        }
        if ((int) allImages.size() <= 1) {
            cout << "No enough jpg images exist in this folder." << endl;
            exit(-1);
        }
        closedir(pdir);
    } else {
        cout << "Error: incorrect jpg folder path." << endl;
        exit(-1);
    }
    sort(allImages.begin(), allImages.end());
    currentImagePtr = allImages.begin();
}

bool ImageStream::read(Mat &m) {
    if(!finish){
        m = imread(*currentImagePtr);
        currentImagePtr++;
        if(*currentImagePtr == allImages.back()){
            finish = true;
        }
        return true;
    } else{
        return false;
    }
}

ImageStream &ImageStream::operator>>(Mat &m) {
    bool flag = read(m);
    return *this;
}

ImageStream::operator bool() {
    return !finish;
}

ImageStream::ImageStream(const ImageStream & ims) = default;

bool ImageStream::isFinish() const {
    return finish;
}

