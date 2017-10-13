//
// Created by xushen on 10/13/17.
//

#include "SLAM.h"

void SLAM::process() {

}

SLAM::SLAM(string settingFile) {
    string inputType, inputPath, calibFile;
    FileStorage fs(settingFile, FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "Could not open the setting file: \"" << settingFile << "\"" << endl;
        exit(-1);
    }
    fs["inputType"] >> inputType;
    fs["inputPath"] >> inputPath;
    fs["calibrationFile"] >> calibFile;

    if(inputType == "image"){
        ms = new ImageStream(inputPath);
    } else if (inputType == "video"){
        ms = new VideoStream(inputPath);
    } else{
        cout << "Invalid input type.";
        exit(-1);
    }

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
}

void SLAM::initialize() {

}
