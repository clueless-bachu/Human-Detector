/**
 *  @file    ioHandler.hpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief IOHandler module handles input data, user options etc
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu
 */
#pragma once
#include <string>
#include <vector>

using std::string;
using std::vector;
namespace vision {

class IOHandler {
 private:
    string outPath;
    string inPath;
    string modelConfigPath = "";
    string modelWeightsPath = "";
    bool isImg = false, ifVisualize = false, record = false;
    double humanHeight = 0;
    vector<double> intrinsicParams;
    vector<double> transform;
    int imgWidth;
    int imgHeight;

 public:
    int yoloWidth = 416;
    int yoloHeight = 416;
    explicit IOHandler(string);
    ~IOHandler();
    void argParse(string);
    bool getInputType();
    bool isVisualize();
    bool ifRecord();
    string getInFilePath();
    string getOutFilePath();
    string getModelConfigPath();
    string getModelWeightsPath();
    double getHumanHeight();
    int getImgWidth();
    int getImgHeight();
    vector<double> getIntrinsics();
    vector<double> getTransform();
    void drawBb(vector<vector<int>>, cv::Mat,  vector<cv::Scalar>);
    static void seeImg(cv::Mat, int);
    void saveImg(string, cv::Mat);
};

}  // namespace vision
