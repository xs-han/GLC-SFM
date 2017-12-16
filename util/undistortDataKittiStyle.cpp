//
// Created by xushen on 11/30/17.
//

#include <iostream>
#include "../VideoStream.h"
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

Mat cameraMatrix;
Mat distortionCoefficient;
Mat m1, m2;
int imageW, imageH;

void setCameraIntrinsicParams(string calibFile) {
    FileStorage fs(calibFile, FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "Could not open the calibration file: \"" << calibFile << "\"" << endl;
        exit(-1);
    }
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distortionCoefficient;

    fs["image_Width"] >> imageW;
    fs["image_Height"] >> imageH;
    initUndistortRectifyMap(cameraMatrix, distortionCoefficient,Mat(), Mat(), Size(imageW, imageH), CV_32F, m1, m2);

    cout << "Set camera martix:" << endl << cameraMatrix << endl;
    cout << "Set distortion coefficients: " << endl << distortionCoefficient << endl;
}

void undistortFrame(Mat &input, Mat &output, bool rectified) {
    if(rectified){
        output = input.clone();
    }else {
        remap(input, output, m1, m2, INTER_CUBIC);
    }
}


int main(int argc, char ** argv){
    if(argc != 4){
        cout << "usage: undistortDataKittiStyle calibXml videoInput folderOutput";
        return -1;
    }
    string calibXml = argv[1];
    string videoInput = argv[2];
    string folderOutput = argv[3];

    setCameraIntrinsicParams(calibXml);

    VideoStream vs(argv[2]);
    Mat frame, rectFrame, outFrame;
    ofstream fout(folderOutput + "/../times.txt");
    int nF = 0;
    while(vs >> frame){
        vs >> frame;
        vs >> frame;
        undistortFrame(frame, rectFrame, false);
        resize(rectFrame,outFrame,Size(640, 360));
        cvtColor( outFrame, outFrame, CV_BGR2GRAY );  //彩色图片转换成黑白图片
        char fileChar[10]; for(int i = 0; i < 10; i++) fileChar[i] = '\0';
        sprintf(fileChar,"%06d",nF);
        string fileName = ""; fileName = fileName + fileChar + ".png";
        cout << "writing file: " << fileName << endl;
        imwrite(folderOutput+"/"+fileName, outFrame);
        fout << vs.id * 1.0 / 30.0 << endl;
        nF += 1;
    }
    fout.close();
    return 0;
}