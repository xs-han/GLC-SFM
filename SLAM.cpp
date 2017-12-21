//
// Created by xushen on 10/13/17.
//

#include "SLAM.h"
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <pangolin/pangolin.h>
#include "Optimizer.h"
#include "DBoW3.h"

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
                    if (matches.size() < 150){
                        float r = matcher.ratio;
                        matcher.setRatio(1);
                        allKeyFrames.back()->isFrameKey(frame, newKps, newDesc, matches);
                        matcher.setRatio(r);
                    }
                    Mat outimg;
                    drawMatches(allKeyFrames.back()->img, allKeyFrames.back()->kps, frame, newKps, matches, outimg);
                    namedWindow("matches",0);
                    imshow("matches", outimg);
                    cvWaitKey(10);
                    //destroyWindow("matches");
                    KeyFrame *k = new KeyFrame(frame, newKps, newDesc);
                    k->time = ms->id * 1.0 / 30.0;
                    track(*k, matches);
                    bool localok = localmap(*k, matches);
                    if(!localok){
                        cerr << "warning: local map failed." << endl;
                        //continue;
                    }

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

                    if(loopClosingType == "Geometric") {
                        if (loopclose(100, 16, matches.size() * 1.5) && 1) {
                            //Optimizer::GlobalBundleAdjustment(allKeyFrames, pointClouds, KeyFrame::cameraMatrix, 30);
                        }
                    } else {
                        if(loopcloseReal(100)){
                            //Optimizer::GlobalBundleAdjustment(allKeyFrames, pointClouds, KeyFrame::cameraMatrix, 30);
                        }
                    }
                }
                if(ms->isFinish()){
                    Optimizer::GlobalBundleAdjustment(allKeyFrames, pointClouds, KeyFrame::cameraMatrix, 10);
                }
            } else{
                imshow("frames", allKeyFrames.back()->img);
                cout << "end frame" << endl;
                if(outRes.is_open()){
                    outRes.close();
                }
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

//        if(generate){
//            generate = false;
//            generateVirtualFrames();
//        }

        // Clear screen and activate view to render into
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        // Render OpenGL Cube
        //pangolin::glDrawColouredCube();\
        //坐标轴的创建
        pangolin::glDrawAxis(3);

        mPaint.drawMap(allKeyFrames, allVirtualFrames, pointClouds, loopPair);
        // Swap frames and Process Events
        pangolin::FinishFrame();

    }
    if(outRes.is_open()){
        outRes.close();
    }
}

SLAM::SLAM(string settingFile) {
    string inputType, inputPath, calibFile, KpsType, descType, vocPath;
    FileStorage fs(settingFile, FileStorage::READ);
    if (!fs.isOpened())
    {
        cout << "Could not open the setting file: \"" << settingFile << "\"" << endl;
        exit(-1);
    }

    fs["inputType"] >> inputType;
    fs["inputPath"] >> inputPath;
    fs["calibrationFile"] >> calibFile;
    fs["KeyPointType"] >> KpsType;
    fs["descriptorType"] >> descType;
    fs["imageScale"] >>imageScale;
    fs["rectified"] >> rectified;
    fs["coloredMap"] >> coloredMap;
    fs["vocPath"] >> vocPath;
    fs["loopClosingType"] >> loopClosingType;

    if(inputType == "image"){
        ms = new ImageStream(inputPath);
    } else if (inputType == "video"){
        ms = new VideoStream(inputPath);
    } else{
        cout << "Invalid input type.";
        exit(-1);
    }

    cout << "Use descriptor " << descType << endl;
    if (descType=="orb")        DescDetector=cv::ORB::create(2000,1.2,6);
    else if (descType=="brisk") DescDetector=cv::BRISK::create();
    else if (descType=="akaze") DescDetector=cv::AKAZE::create();
    else if(descType=="surf" )  DescDetector=cv::xfeatures2d::SURF::create(300, 6, 4, true);
    else if(descType=="surf64" )  DescDetector=cv::xfeatures2d::SURF::create(300, 6, 4);
    else if(descType=="sift" )  DescDetector=cv::xfeatures2d::SIFT::create(1500,6);
    else throw std::runtime_error("Invalid descriptor");
    assert(!descType.empty());
    matcher.setDetecter(KpsDetector);
    matcher.setRatio(0.9);
    KeyFrame::DescDetector = DescDetector;
    KeyFrame::matcher = matcher;

    setCameraIntrinsicParams(calibFile);

    cout << "Use DBoW file: " << vocPath << endl << "Loading ..." << endl;
    voc.load(vocPath);
    db.setVocabulary(voc);
    cout << "Load finished." << endl;

    outRes.open("../cfg/" + descType + "_build_" + loopClosingType + ".txt");
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
    for(int i = 0; i < 20; i++){
        *ms >> frame;
    }
    *ms >> frame;
    undistortFrame(frame, undistFrame);
    frame = undistFrame.clone();
    KeyFrame * k = new KeyFrame(frame);
    k->time = ms->id * 1.0 / 30.0;
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
        if(allKeyFrames.back()->isFrameKeyInit(frame, kps2, desc2, matches)){
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
                cout << "Infeasible essential matrix. Drop this frame." << endl;
                continue;
            }

            assert(cameraMatrix.type() == CV_64F);
            recoverPose(E, points1, points2, R, t, cameraMatrix.at<double>(0,0), Point2d(cameraMatrix.at<double>(0,2), cameraMatrix.at<double>(1,2)), mask);
            cout << t << endl;
            KeyFrame * k1 = new KeyFrame(frame, R, t);
            k1->time = ms->id * 1.0 / 30.0;
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
        resize(output, output, input.size() / imageScale);
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

bool SLAM::localmap(KeyFrame &k, const vector<DMatch> &matches) {
    vector<Point3f> res;
    vector<int> isGood;
    bool ok = allKeyFrames.back()->triangulateNewKeyFrame(k, matches, res, isGood);
    assert(matches.size() == res.size());
    if(1) {
        for (int i = 0; i < res.size(); i++) {

            MapPoint *mp = allKeyFrames.back()->mps[matches[i].queryIdx];
            if (mp == nullptr) {
                if (isGood[i] == 1) {
                    mp = new MapPoint(res[i]);
                    mp->addKf(*allKeyFrames.back(), matches[i].queryIdx, coloredMap);
                    mp->addKf(k, matches[i].trainIdx, coloredMap);
                    pointClouds.push_back(mp);
                    allKeyFrames.back()->mps[matches[i].queryIdx] = mp;
                    k.mps[matches[i].trainIdx] = mp;
                }
            } else {
                mp->addKf(k, matches[i].trainIdx, coloredMap);
                k.mps[matches[i].trainIdx] = mp;
            }
        }
        allKeyFrames.push_back(&k);
        return true;
    }
//    } else{
//        delete &k;
//        KeyFrame::nKeyFrames--;
//        return false;
//    }
}

//void SLAM::generateVirtualFrames() {
//    allVirtualFrames.clear();
//    for(int i = 0; i < allKeyFrames.size(); i++){
//        if(i < 16){
//            continue;
//        }
//        cout << "generating frame: " << i << endl;
//        vector<KeyFrame * > refKfs;
//        for(int j = 0; j < 16; j++){
//            refKfs.push_back(allKeyFrames[i-j]);
//        }
//
//
//        KeyFrame * k = new KeyFrame(*allKeyFrames[i]);
//        Mat relaRvec(3,1,CV_64F), relaTvec(3,1,CV_64F);
//        relaRvec.at<double>(0) = 0;relaRvec.at<double>(1) = CV_PI / 4;relaRvec.at<double>(2) = 0;
//        relaTvec.at<double>(0) = 0;relaTvec.at<double>(1) = 0;relaTvec.at<double>(2) = 0;
//        k->generateRt(allKeyFrames[i], relaRvec, relaTvec);
//        k->generateVisibleMapPoints(refKfs);
//        k->generateImg(refKfs);
//        k->setVirtualFrame(true);
//        vector<KeyFrame * > curVf;
//        curVf.push_back(k);
//        allVirtualFrames.push_back(curVf);
//    }
//}

bool SLAM::loopcloseReal(int delay) {
    bool loopdetected = false;
    KeyFrame & lastKf = *(allKeyFrames.back());
    Mat checkDesc = lastKf.desc.clone();
    QueryResults ret;
    db.query(lastKf.desc.clone(), ret, 20);
    BowVector vkf1; voc.transform(lastKf.desc.clone(), vkf1);
    BowVector vkf2; voc.transform((*(allKeyFrames.end()-2))->desc.clone(), vkf2);
    double r = voc.score(vkf1, vkf2);
    int loopId = -1; double score = -1;
    if(!ret.empty()){
        loopId = ret[0].Id;
        score = ret[0].Score;
        outRes << lastKf.kfId << "\t" << loopId << "\t" << r << "\t" << score << "\t";
        if(score / r >= 0){
            cout << ret[0] << endl;
            KeyFrame * lkf1 = allKeyFrames.back();
            KeyFrame * lkf2 = allKeyFrames[loopId];

            vector<DMatch> mch;
            matcher.match(lkf1->kps, lkf1->desc, lkf2->kps, lkf2->desc, mch);
            allKeyFrames.back()->drawFrameMatches(*lkf1, *lkf2, mch);
            outRes << lkf1->time << "\t" << lkf2->time << "\t";
            outRes << mch.size() << endl;
            if(mch.size() > 20) {
                loopPair.push_back(make_pair(loopId, allKeyFrames.back()->kfId));
                loopdetected = true;
            } else{
                loopdetected = false;
            }
        } else{
            loopdetected = false;
        }
    } else{
        loopdetected = false;
    }

//    if(loopdetected){
//        cout << "get a loop. Pose estimating..." << endl;
//        vector<KeyFrame *> loopRefKf;
//        for(int i = loopId + 4; i > max(loopId - 4, 0); i-- ){
//            loopRefKf.push_back(allKeyFrames[i]);
//        }
//        for(KeyFrame * refkf : loopRefKf){
//            vector<DMatch> refmch;
//            matcher.match(refkf->kps, refkf->desc, lastKf.kps, lastKf.desc, refmch);
//            for(DMatch m:refmch){
//                MapPoint * refp = refkf->mps[m.queryIdx];
//                MapPoint * p = lastKf.mps[m.trainIdx];
//                for(auto ip = pointClouds.end()-1; ip >= pointClouds.begin(); ip--){
//                    if(*ip == p){
//                        pointClouds.erase(ip);
//                        break;
//                    }
//                }
//                if(refp != nullptr && p != nullptr && refp != p){
//                    p->mergePoint(refp);
//                    delete p;
//                }
//            }
//        }
//    }

    if(allKeyFrames.size() < delay + 1){
        return loopdetected;
    } else {
        int nKf = allKeyFrames.size() - 1 - delay;
        KeyFrame * addKf = allKeyFrames[nKf];
        db.add(addKf->desc.clone());
    }
    assert(countNonZero(checkDesc != lastKf.desc) == 0);
    return loopdetected;
}

bool SLAM::loopclose(int delay, int refSize, int threhold) {
    assert(db.size() % 3 == 0);
    bool loopdetected = false;
    KeyFrame & lastKf = *(allKeyFrames.back());
    Mat checkDesc = lastKf.desc.clone();
    QueryResults ret;
    db.query(lastKf.desc.clone(), ret, 20);
    BowVector vkf1; voc.transform(lastKf.desc.clone(), vkf1);
    BowVector vkf2; voc.transform((*(allKeyFrames.end()-2))->desc.clone(), vkf2);
    double r = voc.score(vkf1, vkf2);
    int loopId = -1, virtualId = 0; double score = -1;
    if(!ret.empty()){
        int lid = 0; int nmch = 0;
        for(int i = 0; i < ret.size(); i++) {
            int crtloopId = ret[i].Id / 3;
            int crtvirtualId = ret[i].Id % 3;
            KeyFrame *lkf1 = allKeyFrames.back();
            KeyFrame *lkf2;
            if (crtvirtualId == 0) {
                lkf2 = allKeyFrames[crtloopId];
            } else {
                lkf2 = allVirtualFrames[crtloopId][crtvirtualId - 1];
                if (!lkf2->good) {
                    lkf2 = allKeyFrames[crtloopId];
                }
            }
            vector <DMatch> mch;
            matcher.match(lkf1->kps, lkf1->desc, lkf2->kps, lkf2->desc, mch);
            if(mch.size() > nmch){
                lid = i;
                nmch = mch.size();
                //break;
            }
        }

        loopId = ret[lid].Id / 3;
        virtualId = ret[lid].Id % 3;
        score = ret[lid].Score;
        outRes << lastKf.kfId << "\t" << loopId << "\t" << r << "\t" << score << "\t";

        if (score / r > 0) {
            cout << ret[lid] << endl;
            KeyFrame *lkf1 = allKeyFrames.back();
            KeyFrame *lkf2;
            if (virtualId == 0) {
                lkf2 = allKeyFrames[loopId];
            } else {
                lkf2 = allVirtualFrames[loopId][virtualId - 1];
                if (!lkf2->good) {
                    lkf2 = allKeyFrames[loopId];
                }
            }
            vector <DMatch> mch;
            matcher.match(lkf1->kps, lkf1->desc, lkf2->kps, lkf2->desc, mch);
            outRes << lkf1->time << "\t" << lkf2->time << "\t";
            outRes << mch.size() << endl;
            allKeyFrames.back()->drawFrameMatches(*lkf1, *lkf2, mch);
            if (mch.size() > 20) {
                loopPair.push_back(make_pair(loopId, allKeyFrames.back()->kfId));
                loopdetected = true;
            } else {
                loopdetected = false;
            }
        } else {
            loopdetected = false;
        }
    } else{
        loopdetected = false;
    }
    if(loopdetected){
        cout << "get a loop. Pose estimating..." << endl;
//        vector<KeyFrame *> loopRefKf;
//        for(int i = loopId; i > max(loopId - 4, 0); i-- ){
//            loopRefKf.push_back(allKeyFrames[i]);
//        }
//        for(KeyFrame * refkf : loopRefKf){
//            vector<DMatch> refmch;
//            matcher.match(refkf->kps, refkf->desc, lastKf.kps, lastKf.desc, refmch);
//            for(DMatch m:refmch){
//                MapPoint * p = refkf->mps[m.queryIdx];
//                if(p != nullptr && p->good && p->kfs.back() != &lastKf){
//                    p->addKf(lastKf, m.trainIdx, coloredMap);
//                }
//            }
//        }
    }


    vector<KeyFrame * > curVf;
    for(double angle = -CV_PI / 3; angle <= CV_PI / 3; angle += CV_PI / 3) {
        if(angle == 0){
            continue;
        }
        if(allKeyFrames.size() < refSize){
            KeyFrame *k = new KeyFrame(lastKf, angle);
            k->time = lastKf.time;
            k->good = false;
            curVf.push_back(k);
        } else {
            vector<KeyFrame * > refKfs;
            for(int j = 0; j < refSize; j++){
                refKfs.push_back(*(allKeyFrames.end()-1-j));
            }
            KeyFrame *k = new KeyFrame(lastKf, angle);
            k->time = lastKf.time;
            Mat relaRvec(3, 1, CV_64F), relaTvec(3, 1, CV_64F);
            relaRvec.at<double>(0) = 0;
            relaRvec.at<double>(1) = -angle;
            relaRvec.at<double>(2) = 0;
            relaTvec.at<double>(0) = 0;
            relaTvec.at<double>(1) = 0;
            relaTvec.at<double>(2) = 0;
            k->generateRt(&lastKf, relaRvec, relaTvec);
            k->generateVisibleMapPoints(refKfs);
            cout << "vps: " << k->visibileMps.size() << endl;
            if(k->visibileMps.size() > threhold){
                k->generateImg(refKfs);
                curVf.push_back(k);
                k->good = true;
            } else {
                curVf.push_back(k);
                k->good = false;
            }
        }
    }
    assert(curVf.size() == 2);
    allVirtualFrames.push_back(curVf);
    if(countNonZero(checkDesc != lastKf.desc) != 0){
        cout << "warning! desc is modified" << endl;
        lastKf.desc = checkDesc.clone();
        assert(countNonZero(checkDesc != lastKf.desc) == 0);
    }
    if(allKeyFrames.size() < delay + 1){
        return loopdetected;
    } else {
        int nKf = allKeyFrames.size() - 1 - delay;
        KeyFrame * addKf = allKeyFrames[nKf];
        db.add(addKf->desc.clone());
        for(KeyFrame * vkf : allVirtualFrames[nKf]){
            if(vkf->good) {
                db.add(vkf->desc.clone());
            }else{
                db.add(addKf->desc.clone());
            }
        }
    }
    if(countNonZero(checkDesc != lastKf.desc) != 0){
        cout << "warning! desc is modified again" << endl;
        lastKf.desc = checkDesc.clone();
        assert(countNonZero(checkDesc != lastKf.desc) == 0);
    }
    return loopdetected;
}
