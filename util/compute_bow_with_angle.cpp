//
// Created by xushen on 11/20/17.
//

#include <iostream>
#include "DBoW3.h"
#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core.hpp>

#include "../VideoStream.h"

using namespace std;
using namespace DBoW3;
using namespace cv;


Mat cameraMatrix, distortionCoefficient;
int imageWidth, imageHeight;
Mat m1, m2;

void setCameraIntrinsicParams(string calibFile) {
    FileStorage fs(calibFile, FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "Could not open the calibration file: \"" << calibFile << "\"" << endl;
        exit(-1);
    }
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distortionCoefficient;

    fs["image_Width"] >> imageWidth;
    fs["image_Height"] >> imageHeight;
    initUndistortRectifyMap(cameraMatrix, distortionCoefficient,Mat(), Mat(), Size(imageWidth, imageHeight), CV_32F, m1, m2);

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
    Vocabulary voc(argv[1]);
    Database db(voc);
    ofstream fout(argv[2]);
    setCameraIntrinsicParams("../cfg/calib_gopro_wide.xml");
    vector<string> videos;
    for(int i = 3; i < argc; i++){
        videos.push_back(string(argv[i]));
    }
    sort(videos.begin(), videos.end());

    cv::Ptr<cv::Feature2D> fdetector;
    fdetector=cv::xfeatures2d::SURF::create(300, 6, 4);

    cout << "Processing video: " << videos[0] << endl;
    VideoStream vs_0(videos[0]);
    Mat im_0, im_ref;
    for(int i = 0; i < 30; i++) vs_0 >> im_ref;
    vs_0 >> im_0;

    undistortFrame(im_0, im_0, 0); resize(im_0,im_0,im_0.size()/2);
    vector<KeyPoint> key_0; Mat desc_0;
    fdetector->detectAndCompute(im_0,noArray(), key_0, desc_0);
    Mat out; drawKeypoints(im_0,key_0,out);
    namedWindow("outkey"); imshow("outkey",out); cvWaitKey(0);
    db.add(desc_0);

    undistortFrame(im_ref, im_ref, 0); resize(im_ref,im_ref,im_ref.size()/2);
    vector<KeyPoint> key_ref; Mat desc_ref;
    fdetector->detectAndCompute(im_ref,noArray(), key_ref, desc_ref);
    drawKeypoints(im_ref,key_ref,out);
    imwrite("../jpg/"+to_string(0)+".jpg", out);
    //namedWindow("outkey"); imshow("outkey",out); cvWaitKey(0);
    QueryResults res;
    db.query(desc_ref,res,1);

    fout << videos[0] << " " << res[0] << endl;
    for(int i = 1; i < videos.size(); i++){
        cout << "Processing video " << videos[i] << endl;
        VideoStream vs_angle(videos[i]);

        Mat im_angle;for(int j = 0; j < 30; j++) vs_angle >> im_angle;
        undistortFrame(im_angle, im_angle, 0); resize(im_angle,im_angle,im_angle.size()/2);

        vector<KeyPoint> key_angle; Mat desc_angle;
        fdetector->detectAndCompute(im_angle,noArray(), key_angle, desc_angle);
        Mat out; drawKeypoints(im_angle,key_angle,out);
        imwrite("../jpg/"+to_string(i)+".jpg", out);
        namedWindow("outkey"); imshow("outkey",out); cvWaitKey(0);
        QueryResults res1;
        db.query(desc_angle,res1,1);
        fout << videos[i] << " " << res1[0] << endl;
    }

    fout.close();

    return 0;
}