//
// Created by xushen on 10/13/17.
//

#include "SLAM.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>

void SLAM::process() {
    Mat frame;
    namedWindow("frames", 0);
    initialize();
    while (*ms >> frame){
        Mat undistFrame;
        undistortFrame(frame, undistFrame);
        imshow("frames", undistFrame);
        cvWaitKey(0);
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
    fs["Camera_Matrix"] >> cameraMatrix;
    fs["Distortion_Coefficients"] >> distortionCoefficient;

    KeyFrame::cameraMatrix = cameraMatrix.clone();
    cout << "Set camera martix:" << endl << cameraMatrix << endl;
    cout << "Set distortion coefficients: " << endl << distortionCoefficient << endl;
}

void SLAM::initialize() {
    Mat frame;
    Mat undistFrame;
    // wait first 1s;
//    for(int i = 0; i < 31; i++){
//        *ms >> frame;
//    }
    *ms >> frame;
    undistortFrame(frame, undistFrame);
    frame = undistFrame.clone();
    KeyFrame * k = new KeyFrame(frame);
    allKeyFrames.push_back(k);

    while (true){
        if(ms->isFinish()){
            cout << "Initialize failed. Detect end of the input media stream." << endl;
            exit(-1);
        }
        *ms >> frame;
        undistortFrame(frame, undistFrame);
        frame = undistFrame.clone();

        vector<Point2f> points1;
        vector<Point2f> points2;
        vector<DMatch> matches;
        vector<KeyPoint> & kps1 = allKeyFrames.back()->kps;
        Mat & desc1 = allKeyFrames.back()->desc;
        vector<KeyPoint> kps2;
        Mat desc2;
        Mat R, t;
        if(allKeyFrames.back()->isFrameKey(frame, kps2, desc2, matches)){
            points1.clear();points2.clear();
            points1.resize((int)matches.size()); points2.resize((int)matches.size());
            int i = 0;
            for(DMatch m : matches){
                points1[i] = kps1[m.queryIdx].pt;
                points2[i] = kps2[m.trainIdx].pt;
                i++;
            }
            Mat E = findEssentialMat(points1, points2, cameraMatrix);
            recoverPose(E, points1, points2, R, t, cameraMatrix.at<float>(1,1), Point2f(cameraMatrix.at<float>(0,2), cameraMatrix.at<float>(1,2)), noArray());
            KeyFrame * k1 = new KeyFrame(frame, R, t);

            allKeyFrames.push_back(k1);
            break;
        }
    }

}

void SLAM::undistortFrame(Mat &input, Mat &output) {
    Mat m1, m2;
    initUndistortRectifyMap(cameraMatrix, distortionCoefficient, Mat::eye(3,3,CV_32F), cameraMatrix, input.size(), CV_32FC1, m1, m2);
    remap(input, output, m1, m2, INTER_CUBIC);
}
