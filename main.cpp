#include <iostream>

#include "ImageStream.h"
#include "VideoStream.h"

using namespace cv;
using namespace std;

int main() {
    MediaStream * ms = new VideoStream("/home/xushen/Videos/GOPR0015.MP4");
    Mat m, m2;
    namedWindow("test", 0);
    while(*(ms) >> m){
        imshow("test",m);
        cvWaitKey(30);
    }
    std::cout << "Hello, World!" << std::endl;
    return 0;
}