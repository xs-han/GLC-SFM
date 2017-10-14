//
// Created by xushen on 10/14/17.
//

#include "KeyFrame.h"
#include <iostream>
#include <opencv2/xfeatures2d/nonfree.hpp>
using namespace std;
using namespace cv;

Mat KeyFrame::cameraMartix;
cv::Ptr<cv::Feature2D> KeyFrame::detector;
FeatureMatcher KeyFrame::matcher;