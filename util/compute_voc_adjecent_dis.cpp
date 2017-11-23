//
// Created by xushen on 11/20/17.
//

#include <iostream>
#include "DBoW3.h"
#include <opencv2/opencv.hpp>
using namespace std;
using namespace DBoW3;
using namespace cv;

int main(int argc, char ** argv){
    if(argc != 3) {
        cout << "Usage: compute_voc_ajdecent_dis vocFile outputFile" << endl;
        exit(-1);
    }

    Vocabulary voc(argv[1]);
    ofstream fout(argv[2]);

    vector<Mat> allWords;
    for(int i = 0; i < voc.m_words.size(); i++){
        if(!voc.m_words[i]->isLeaf()){
            cerr << "error word" << endl;
            return -1;
        }
        allWords.push_back(voc.m_words[i]->descriptor);
    }

    double dis = 10000000000;
    double newDis = -1;
    int id = -1;
    for(int i = 0; i < allWords.size(); i++){
        id = -1;
        dis = 10000000000;
        if(i % 100 == 0) {
            cout << "compute feature: " << i << "/" << allWords.size() << endl;
        }
        for(int j = max(0, i - 3000); j < min((int)allWords.size(), i+3000); j++){
            if(i == j)
                continue;
            newDis = norm(allWords[i], allWords[j]);
            if(newDis < dis){
                dis = newDis;
                id = j;
            }
        }
        fout << i << " " << id << " " << dis << endl;
    }

    fout.close();

    return 0;
}