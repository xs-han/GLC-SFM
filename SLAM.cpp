//
// Created by xushen on 10/13/17.
//

#include "SLAM.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <pangolin/pangolin.h>

void SLAM::process() {
    Mat frame;
    namedWindow("frames", 0);
    initialize();
    //mPaint.drawMap(allKeyFrames, pointClouds);
    //创建一个窗口
    pangolin::CreateWindowAndBind("Main",640,480);
    //启动深度测试
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(640,480,420,420,320,240,0.2,100),
            //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
            pangolin::ModelViewLookAt(0,-10,0.1,0,0,0,pangolin::AxisNegY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    //setBounds 跟opengl的viewport 有关
    //看SimpleDisplay中边界的设置就知道
    pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0,1.0,0.0,1.0,-640.0f/480.0f)
            .SetHandler(&handler);

    while (!pangolin::ShouldQuit()){
        mPaint.drawMap(allKeyFrames, pointClouds);
        if(*ms >> frame) {
            Mat undistFrame;
            undistortFrame(frame, undistFrame);
            imshow("frames", undistFrame);
            cvWaitKey(10);
        }
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
        //pangolin::glDrawColouredCube();\
        //坐标轴的创建
        pangolin::glDrawAxis(3);

        mPaint.drawMap(allKeyFrames, pointClouds);
        // Swap frames and Process Events
        pangolin::FinishFrame();
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
    //cameraMatrix.convertTo(cameraMatrix, CV_32F);
    //distortionCoefficient.convertTo(distortionCoefficient,CV_32F);

    KeyFrame::cameraMatrix = cameraMatrix.clone();
    cout << "Set camera martix:" << endl << cameraMatrix << endl;
    cout << "Set distortion coefficients: " << endl << distortionCoefficient << endl;
}

void SLAM::initialize() {
    Mat frame;
    Mat undistFrame;
    // wait first 1s;
    for(int i = 0; i < 31; i++){
        *ms >> frame;
    }
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
        Mat res;
        vector<DMatch> matches;
        vector<KeyPoint> & kps1 = allKeyFrames.back()->kps;
        Mat & desc1 = allKeyFrames.back()->desc;
        vector<KeyPoint> kps2;
        Mat desc2;
        Mat R, t, mask;
        if(allKeyFrames.back()->isFrameKey(frame, kps2, desc2, matches)){
            Mat outimg;
            drawMatches(allKeyFrames.back()->img, allKeyFrames.back()->kps, frame, kps2, matches, outimg);
            namedWindow("matches",0);
            imshow("matches", outimg);
            cvWaitKey(0);
            destroyWindow("matches");

            points1.clear();points2.clear();
            points1.resize((int)matches.size()); points2.resize((int)matches.size());
            int i = 0;
            for(DMatch m : matches){
                points1[i] = kps1[m.queryIdx].pt;
                points2[i] = kps2[m.trainIdx].pt;
                i++;
            }
            Mat E = findEssentialMat(points1, points2, cameraMatrix.at<double>(0,0), Point2d(cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2)), RANSAC, 0.999, 1, mask);
            double feasible_count = countNonZero(mask);
            cout << (int)feasible_count << " -in- " << points1.size() << endl;
            recoverPose(E, points1, points2, R, t, cameraMatrix.at<double>(0,0), Point2d(cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2)), mask);
            KeyFrame * k1 = new KeyFrame(frame, R, t);
            allKeyFrames.back()->triangulateNewKeyFrame(*k1, matches, res);
            assert((int)matches.size() == res.cols);
            assert(res.type() == CV_32F);
            cout << res << endl;
            for(i = 0; i < res.cols; i++){
                Mat p = res.col(i);
                Point3f pp((p.at<float>(0) / p.at<float>(3)),
                           (p.at<float>(1) / p.at<float>(3)),
                           (p.at<float>(2) / p.at<float>(3)));

                MapPoint * mp = new MapPoint(pp);
                mp->addKf(*allKeyFrames.back(), allKeyFrames.back()->kps[matches[i].queryIdx]);
                mp->addKf(*k1, k1->kps[matches[i].trainIdx]);
                pointClouds.push_back(mp);
                allKeyFrames.back()->mps[matches[i].queryIdx] = mp;
                k1->mps[matches[i].trainIdx] = mp;
            }
            allKeyFrames.push_back(k1);

            break;
        }
    }

}

void SLAM::undistortFrame(Mat &input, Mat &output) {
    Mat m1, m2;
    initUndistortRectifyMap(cameraMatrix, distortionCoefficient, Mat::eye(3,3,CV_32F), cameraMatrix, input.size(), CV_32F, m1, m2);
    remap(input, output, m1, m2, INTER_CUBIC);
}
