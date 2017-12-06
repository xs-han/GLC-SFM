//
// Created by xushen on 11/28/17.

#include <opencv2/core.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <fstream>
#include "../FeatureMatcher.h"
using namespace std;
using namespace cv;

int main(int argc, char ** argv){
    if(argc != 4){
        cout << "usage: compute_tracked_desc_dis descType image1 image2" << endl;
        return -1;
    }

    string descriptor = argv[1];
    ofstream fout("../cfg/"+descriptor+"_dis.txt");

    FeatureMatcher fm;

    //select detector
    cv::Ptr<cv::Feature2D> fdetector;
    if (descriptor=="orb")        fdetector=cv::ORB::create(2000);
    else if (descriptor=="brisk") fdetector=cv::BRISK::create();
    else if(descriptor=="surf" )  fdetector=cv::xfeatures2d::SURF::create(300, 6, 4, true);
    else if(descriptor=="surf64" )  fdetector=cv::xfeatures2d::SURF::create(300, 6, 4);
    else if(descriptor=="sift" )  fdetector=cv::xfeatures2d::SIFT::create(1500, 6);
    else throw std::runtime_error("Invalid descriptor");
    assert(!descriptor.empty());

    fm.setDetecter(fdetector);
    fm.setRatio(1);

    Mat img1 = imread(argv[2], CV_LOAD_IMAGE_GRAYSCALE); vector<KeyPoint> keypoints1; Mat descriptors1;
    fdetector->detectAndCompute(img1, cv::Mat(), keypoints1, descriptors1);

    Mat img2 = imread(argv[3], CV_LOAD_IMAGE_GRAYSCALE); vector<KeyPoint> keypoints2; Mat descriptors2;
    fdetector->detectAndCompute(img2, cv::Mat(), keypoints2, descriptors2);

    vector<DMatch> mch;

    fm.match(keypoints1,descriptors1,keypoints2,descriptors2, mch);
//    cv::Ptr<cv::DescriptorMatcher > matcher;
//    vector<vector<DMatch> > m12, m21;
//    matcher.reset(new cv::BFMatcher(cv::NORM_L2));
//    matcher->knnMatch(descriptors1,
//                      descriptors2,
//                      m12, // vector of matches (up to 2 per entry)
//                      2);        // return 2 nearest neighbours
//    matcher->knnMatch(descriptors2,
//                      descriptors1,
//                      m21, // vector of matches (up to 2 per entry)
//                      2);        // return 2 nearest neighbours
//    fm.symmetryTest(m12,m21,mch);

//    for(vector<DMatch> m : m12){
//        mch.push_back(m[0]);
//    }

    if(mch.size() == 0){
        cout << "no match" << endl;
        return  -1;
    }
    double totalDis = 0;
    for(int i = 0; i < mch.size(); i++){
        Mat desc1 = descriptors1.row(mch[i].queryIdx);
        Mat desc2 = descriptors2.row(mch[i].trainIdx);

        totalDis += norm(desc1, desc2);
        fout << norm(desc1, desc2) << endl;
    }
    cout << "average distance: " << totalDis / (double)mch.size() << endl;
    fout.close();

    Mat out;
    drawMatches(img1,keypoints1,img2,keypoints2,mch,out);
    namedWindow("mch");
    imshow("mch", out);
    imwrite("../jpg/dispairty12.jpg",out);
    cvWaitKey(0);

}