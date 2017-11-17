#define USE_CONTRIB
//First step of creating a vocabulary is extracting features from a set of images. We save them to a file for next step
#include <iostream>
#include <vector>

// DBoW3
#include "DBoW3.h"

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#ifdef USE_CONTRIB
#include <opencv2/xfeatures2d/nonfree.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#endif
#include "DescManip.h"

using namespace DBoW3;
using namespace std;
using namespace cv;


Mat cameraMatrix;
Mat distortionCoefficient;
Mat m1, m2;
int imageWidth, imageHeight;

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

//command line parser
class CmdLineParser{int argc; char **argv; public: CmdLineParser(int _argc,char **_argv):argc(_argc),argv(_argv){}  bool operator[] ( string param ) {int idx=-1;  for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i;    return ( idx!=-1 ) ;    } string operator()(string param,string defvalue="-1"){int idx=-1;    for ( int i=0; i<argc && idx==-1; i++ ) if ( string ( argv[i] ) ==param ) idx=i; if ( idx==-1 ) return defvalue;   else  return ( argv[  idx+1] ); }};


// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

// extended surf gives 128-dimensional vectors
const bool EXTENDED_SURF = true;
// - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - 

void wait()
{
    cout << endl << "Press enter to continue" << endl;
    getchar();
}


vector<string> readImagePaths(int argc,char **argv,int start){
    vector<string> paths;
    for(int i=start;i<argc;i++)    paths.push_back(argv[i]);
        return paths;
}

vector< cv::Mat  >  loadFeatures( std::vector<string> path_to_images,int delay,string descriptor="") throw (std::exception){
    //select detector
    cv::Ptr<cv::Feature2D> fdetector;
    if (descriptor=="orb")        fdetector=cv::ORB::create();
    else if (descriptor=="brisk") fdetector=cv::BRISK::create();
#ifdef OPENCV_VERSION_3
    else if (descriptor=="akaze") fdetector=cv::AKAZE::create();
#endif
#ifdef USE_CONTRIB
    else if(descriptor=="surf" )  fdetector=cv::xfeatures2d::SURF::create(300, 6, 4, EXTENDED_SURF);
#endif

    else throw std::runtime_error("Invalid descriptor");
    assert(!descriptor.empty());
    vector<cv::Mat>    features;


    cout << "Extracting   features..." << endl;
    namedWindow("img");
    for(size_t i = 0; i < path_to_images.size(); i += delay)
    {
        vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        cout<<"reading image: "<<path_to_images[i]<<endl;
        cv::Mat image = cv::imread(path_to_images[i], 0);
        if(image.empty())throw std::runtime_error("Could not open image"+path_to_images[i]);
        Mat rectImage = image.clone();
        //undistortFrame(image,rectImage, false);
        imshow("img", rectImage);
        cvWaitKey(10);
        cout<<"extracting features"<<endl;
        fdetector->detectAndCompute(rectImage, cv::Mat(), keypoints, descriptors);
        features.push_back(descriptors);
        cout<<"done detecting " << keypoints.size() << " features"<<endl;
    }
    return features;
}

// ----------------------------------------------------------------------------
void saveToFile(string filename,const vector<cv::Mat> &features){

    //test it is not created
    std::ifstream ifile(filename);
    if (ifile.is_open()){cerr<<"ERROR::: Output File "<<filename<<" already exists!!!!!"<<endl;exit(0);}
    std::ofstream ofile(filename);
    if (!ofile.is_open()){cerr<<"could not open output file"<<endl;exit(0);}
    uint32_t size=features.size();
    ofile.write((char*)&size,sizeof(size));
    for(auto &f:features){
        if( !f.isContinuous()){
            cerr<<"Matrices should be continuous"<<endl;exit(0);
        }
        uint32_t aux=f.cols; ofile.write( (char*)&aux,sizeof(aux));
          aux=f.rows; ofile.write( (char*)&aux,sizeof(aux));
          aux=f.type(); ofile.write( (char*)&aux,sizeof(aux));
        ofile.write( (char*)f.ptr<uchar>(0),f.total()*f.elemSize());
    }
}

// ----------------------------------------------------------------------------

int main(int argc,char **argv)
{

    try{
        CmdLineParser cml(argc,argv);
        if (cml["-h"] || argc==1){
            cerr<<"Usage:  descriptor_name output image0 image1 ... \n\t descriptors:brisk,surf,orb(default),akaze(only if using opencv 3)"<<endl;
            return -1;
        }
        setCameraIntrinsicParams("../cfg/Bicocca.xml");

        string descriptor=argv[1];
        string output=argv[2];

        auto images=readImagePaths(argc,argv,3);
        vector< cv::Mat   >   features= loadFeatures(images,20,descriptor);

      //save features to file
    saveToFile(argv[2],features);

    }catch(std::exception &ex){
        cerr<<ex.what()<<endl;
    }

    return 0;
}
