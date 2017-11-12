//
// Created by xushen on 10/13/17.
//

#include "SLAM.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <pangolin/pangolin.h>
#include "Optimizer.h"

void SLAM::process() {

    namedWindow("frames", 0);
    initialize();
    //mPaint.drawMap(allKeyFrames, pointClouds);
    //创建一个窗口
    pangolin::CreateWindowAndBind("Main",640,480);

    //启动深度测试
    glEnable(GL_DEPTH_TEST);

    // Define Projection and initial ModelView matrix
    pangolin::OpenGlRenderState s_cam(
            pangolin::ProjectionMatrix(1920,1080,1286,1286,978,614,0.2,10000),
            //对应的是gluLookAt,摄像机位置,参考点位置,up vector(上向量)
            pangolin::ModelViewLookAt(0,-10,0.1,0,0,0,pangolin::AxisNegY)
    );

    // Create Interactive View in window
    pangolin::Handler3D handler(s_cam);
    //setBounds 跟opengl的viewport 有关
    //看SimpleDisplay中边界的设置就知道
    pangolin::View &d_cam = pangolin::CreateDisplay().SetBounds(0.0,1.0,0.0,1.0,-640.0f/480.0f)
            .SetHandler(&handler);

    bool wait = false;
    bool quit = false;
    bool generate = false;
    while (!pangolin::ShouldQuit()){
        Mat frame;
        if(!wait && !quit){
            if(*ms >> frame) {
                Mat undistFrame;
                undistortFrame(frame, undistFrame);
                frame = undistFrame;
                imshow("frames", frame);
                char key = cvWaitKey(10);
                if(key == 'w') wait = true;
                if(key == 'q') quit = true;

                vector<KeyPoint> newKps;
                Mat newDesc;
                vector<DMatch> matches;
                if (allKeyFrames.back()->isFrameKey(frame, newKps, newDesc, matches)) {
                    cout << "Number of matches:" << matches.size() << endl;
//                    Mat outimg;
//                    drawMatches(allKeyFrames.back()->img, allKeyFrames.back()->kps, frame, newKps, matches, outimg);
//                    namedWindow("matches",0);
//                    imshow("matches", outimg);
//                    cvWaitKey(0);
//                    destroyWindow("matches");
                    KeyFrame *k = new KeyFrame(frame, newKps, newDesc);
                    track(*k, matches);
                    localmap(*k, matches);

                    vector<KeyFrame * > localFrames;
                    vector<MapPoint * > localPoints;
                    localFrames.push_back(*(allKeyFrames.end()-3));
                    localFrames.push_back(*(allKeyFrames.end()-2));
                    localFrames.push_back(*(allKeyFrames.end()-1));
                    for(KeyFrame * kf: localFrames){
                        for(MapPoint * p : kf->mps) {
                            if(p== nullptr || (!p->good)){
                                continue;
                            }
                            if(find(localPoints.begin(), localPoints.end(), p) == localPoints.end()) {
                                localPoints.push_back(p);
                            }
                        }
                    }

                    Optimizer::GlobalBundleAdjustment(localFrames, localPoints, KeyFrame::cameraMatrix, 10);

                    if(allKeyFrames.back()->kfId % 100 == 0){
                        Optimizer::GlobalBundleAdjustment(allKeyFrames, pointClouds, KeyFrame::cameraMatrix, 10);
                    }
                }
                if(ms->isFinish()){
                    Optimizer::GlobalBundleAdjustment(allKeyFrames, pointClouds, KeyFrame::cameraMatrix, 10);
                }
            } else{
                imshow("frames", allKeyFrames.back()->img);
                char key = cvWaitKey(10);
                if(key == 'w') wait = true;
                if(key == 'q') quit = true;
            }
        }
        else{
            char key = cvWaitKey(10);
            if(key == 'w') wait = false;
            if(key == 'g') generate = true;
        }

        if(generate){
            generate = false;
            generateVirtualFrames();
        }
        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
        //pangolin::glDrawColouredCube();\
        //坐标轴的创建
        pangolin::glDrawAxis(3);

        vector<KeyFrame *> allDrawFrames;
        allDrawFrames.insert(allDrawFrames.end(), allKeyFrames.begin(), allKeyFrames.end());
        if(!allVirtualFrames.empty()){
            allDrawFrames.insert(allDrawFrames.end(), allVirtualFrames.begin(), allVirtualFrames.end());
        }
        mPaint.drawMap(allDrawFrames, pointClouds);
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
    fs["imageScale"] >>imageScale;
    fs["rectified"] >> rectified;
    fs["coloredMap"] >> coloredMap;

    if(inputType == "image"){
        ms = new ImageStream(inputPath);
    } else if (inputType == "video"){
        ms = new VideoStream(inputPath);
    } else{
        cout << "Invalid input type.";
        exit(-1);
    }

    cout << "Use descriptor " << descType << endl;
    if (descType=="orb")        detector=cv::ORB::create(2500, 1.2, 8, 20);
    else if (descType=="brisk") detector=cv::BRISK::create();
    else if (descType=="akaze") detector=cv::AKAZE::create();
    else if(descType=="surf" )  detector=cv::xfeatures2d::SURF::create(300, 8, 6);
    else if(descType=="sift" )  detector=cv::xfeatures2d::SIFT::create(500);
    else throw std::runtime_error("Invalid descriptor");
    assert(!descType.empty());
    matcher.setDetecter(detector);
    matcher.setRatio(1);
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

    int imageWidth, imageHeight;
    fs["image_Width"] >> imageWidth;
    fs["image_Height"] >> imageHeight;
    Size inputSize(imageWidth, imageHeight);
    newCameraMatrix = getOptimalNewCameraMatrix(cameraMatrix, distortionCoefficient, inputSize, 1, inputSize);
    cout << cameraMatrix << endl << newCameraMatrix << endl;
    initUndistortRectifyMap(cameraMatrix, distortionCoefficient,Mat(), Mat(), inputSize, CV_32F, m1, m2);

    Mat frameCameraMatrix = cameraMatrix.clone() / imageScale;
    frameCameraMatrix.at<double>(2,2) = 1;
    KeyFrame::cameraMatrix = frameCameraMatrix.clone();
    cameraMatrix = frameCameraMatrix.clone();
    cout << "Set camera martix:" << endl << KeyFrame::cameraMatrix << endl;
    cout << "Set distortion coefficients: " << endl << distortionCoefficient << endl;
}

void SLAM::initialize() {
    Mat frame;
    Mat undistFrame;
    // Drop first unstable 30 frame;
    for(int i = 0; i < 30; i++){
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

        vector<Point3f> res; vector<int> isGood;
        vector<DMatch> matches;
        vector<KeyPoint> & kps1 = allKeyFrames.back()->kps, kps2;
        Mat & desc1 = allKeyFrames.back()->desc, desc2;
        Mat R, t, mask;
        if(allKeyFrames.back()->isFrameKey(frame, kps2, desc2, matches)){
//            Mat outimg;
//            drawMatches(allKeyFrames.back()->img, allKeyFrames.back()->kps, frame, kps2, matches, outimg);
//            namedWindow("matches",0);
//            imshow("matches", outimg);
//            cvWaitKey(0);
//            destroyWindow("matches");
            vector<Point2f> points1(matches.size());
            vector<Point2f> points2(matches.size());
            int i = 0;
            for(DMatch m : matches){
                points1[i] = kps1[m.queryIdx].pt;
                points2[i] = kps2[m.trainIdx].pt;
                i++;
            }
            Mat E = findEssentialMat(points1, points2, cameraMatrix.at<double>(0,0), Point2d(cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2)), RANSAC, 0.999, 1, mask);
            // Check EssentialMat correctness
            int feasible_count = countNonZero(mask);
            cout << feasible_count << " -in- " << matches.size() << endl;
            if(feasible_count < matches.size() * 0.3){
                cout << "Infeasible  essential matrix. Drop this frame." << endl;
                continue;
            }

            assert(cameraMatrix.type() == CV_64F);
            recoverPose(E, points1, points2, R, t, cameraMatrix.at<double>(0,0), Point2d(cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2)), mask);
            cout << t << endl;
            KeyFrame * k1 = new KeyFrame(frame, R, t);
            allKeyFrames.back()->triangulateNewKeyFrame(*k1, matches, res, isGood);
            assert((int)matches.size() == res.size());

            for(i = 0; i < res.size(); i++){
                if(isGood[i] == 1) {
                    MapPoint *mp = new MapPoint(res[i]);
                    mp->addKf(*allKeyFrames.back(), matches[i].queryIdx, coloredMap);
                    mp->addKf(*k1, matches[i].trainIdx, coloredMap);
                    pointClouds.push_back(mp);
                    allKeyFrames.back()->mps[matches[i].queryIdx] = mp;
                    k1->mps[matches[i].trainIdx] = mp;
                }
            }
            allKeyFrames.push_back(k1);

            break;
        }
    }

}

void SLAM::undistortFrame(Mat &input, Mat &output) {
    if(rectified){
        output = input.clone();
    }else {
        remap(input, output, m1, m2, INTER_CUBIC);
    }
    if(imageScale != 1){
        resize(output, output, input.size()/imageScale);
    }
}

void SLAM::track(KeyFrame & k, const vector <DMatch> & matches) {
    vector<int> inliers;
    allKeyFrames.back()->computeNewKfRT(k, matches, inliers);
//    int nextInlier = inliers[0], j = 0;
//
//    for(int i = 0; i < matches.size(); i++){
//        if(i == nextInlier){
//            j+=1;
//            if(j < inliers.size()) {
//                nextInlier = inliers[j];
//            }
//        } else{
//            MapPoint * mp = allKeyFrames.back()->mps[matches[i].queryIdx];
//            if(mp != nullptr){
//                mp->correctMapPoint(&k, matches[i].trainIdx);
//            }
//        }
//    }
//    inliers.clear();
//    allKeyFrames.back()->computeNewKfRT(k, matches, inliers);
    cout << endl << "match size: " << matches.size() << ", inlier size: " << inliers.size() << endl;
}

void SLAM::localmap(KeyFrame &k, const vector<DMatch> &matches) {
    vector<Point3f> res;
    vector<int> isGood;
    allKeyFrames.back()->triangulateNewKeyFrame(k, matches, res, isGood);
    assert(matches.size() == res.size());

    for(int i = 0; i < res.size(); i++){

        MapPoint * mp = allKeyFrames.back()->mps[matches[i].queryIdx];
        if(mp == nullptr){
            if(isGood[i] == 1) {
                mp = new MapPoint(res[i]);
                mp->addKf(*allKeyFrames.back(), matches[i].queryIdx, coloredMap);
                mp->addKf(k, matches[i].trainIdx, coloredMap);
                pointClouds.push_back(mp);
                allKeyFrames.back()->mps[matches[i].queryIdx] = mp;
                k.mps[matches[i].trainIdx] = mp;
            }
        } else{
            mp->addKf(k, matches[i].trainIdx, coloredMap);
            k.mps[matches[i].trainIdx] = mp;
        }
    }
    allKeyFrames.push_back(&k);
}

void SLAM::generateVirtualFrames() {
    allVirtualFrames.clear();
    for(int i = 0; i < allKeyFrames.size(); i++){
        if(i < 16){
            continue;
        }
        cout << "generating frame: " << i << endl;
        KeyFrame * k = new KeyFrame(*allKeyFrames[i]);
        Mat relaRvec(3,1,CV_64F), relaTvec(3,1,CV_64F);
        relaRvec.at<double>(0) = 0;relaRvec.at<double>(1) = CV_PI / 4;relaRvec.at<double>(2) = 0;
        relaTvec.at<double>(0) = 0;relaTvec.at<double>(1) = 0;relaTvec.at<double>(2) = 0;
        k->generateRt(allKeyFrames[i], relaRvec, relaTvec);

        vector<KeyFrame * > refKfs;
        for(int j = 0; j < 16; j++){
            refKfs.push_back(allKeyFrames[i-j]);
        }

        k->generateVisibleMapPoints(refKfs);
        k->generateImg(refKfs);
        allVirtualFrames.push_back(k);
        cout << "OK" << endl;
    }
}
