#include <iostream>

#include "SLAM.h"

using namespace cv;
using namespace std;

int main() {
    SLAM mvslam("../cfg/setting.xml");
    mvslam.process();
    std::cout << "Hello, World!" << std::endl;
    return 0;
}