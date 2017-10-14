//
// Created by xushen on 10/13/17.
//

#include "SLAM.h"

void SLAM::process() {
    Mat frame;
    namedWindow("frames", 0);
    while (*ms >> frame){
        Mat undistFrame;
        undistortFrame(frame, undistFrame);
        imshow("frames", undistFrame);
        cvWaitKey(20);
    }
}

SLAM::SLAM(string settingFile) {
    string inputType, inputPath, calibFile, descType;
    FileStorage fs(settingFile, FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "Could not open the setting file: \"" << settingFile << "\"" << endl;
        exit(-1);
    }
    fs["inputType"] >> inputType;
    fs["inputPath"] >> inputPath;
    fs["calibrationFile"] >> calibFile;
    fs["descriptorType"] >> descType;

    if(inputType == "image"){
        ms = new ImageStream(inputPath);
    } else if (inputType == "video"){
        ms = new VideoStream(inputPath);
    } else{
        cout << "Invalid input type.";
        exit(-1);
    }

    cout << "Use descriptor " << descType << endl;
    if (descType=="orb")        detector=cv::ORB::create(500);
    else if (descType=="brisk") detector=cv::BRISK::create();
    else if (descType=="akaze") detector=cv::AKAZE::create();
    else if(descType=="surf" )  detector=cv::xfeatures2d::SURF::create(500);
    else if(descType=="sift" )  detector=cv::xfeatures2d::SIFT::create(500);
    else throw std::runtime_error("Invalid descriptor");
    assert(!descType.empty());
    matcher.setDetecter(detector);
    KeyFrame::detector = detector;
    KeyFrame::matcher = matcher;

    setCameraIntrinsicParams(calibFile);
}

void SLAM::setCameraIntrinsicParams(string calibFile) {
    FileStorage fs(calibFile, FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "Could not open the calibration file: \"" << calibFile << "\"" << endl;
        exit(-1);
    }
    fs["Camera_Matrix"] >> cameraMartix;
    fs["Distortion_Coefficients"] >> distortionCoefficient;

    KeyFrame::cameraMartix = cameraMartix.clone();
    cout << "Set camera martix:" << endl << cameraMartix << endl;
    cout << "Set distortion coefficients: " << endl << distortionCoefficient << endl;
}

void SLAM::initialize() {

}

void SLAM::undistortFrame(Mat &input, Mat &output) {
    Mat map1, map2;
    Mat R = Mat::eye(3,3,CV_32F);
    initUndistortRectifyMap(cameraMartix, distortionCoefficient, R,
                            getOptimalNewCameraMatrix(cameraMartix, distortionCoefficient, Size(640, 480), 1, Size(320, 240), 0)
            , Size(320, 240), CV_32FC1, map1, map2);
    output = input.clone();
    remap(input, output, map1, map2, INTER_CUBIC);
}


