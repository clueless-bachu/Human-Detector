/**
*  @file    ioHandling.cpp
*  @author  Vasista (clueless-bachu)
*  @author  Vishnuu (vishnuu95)
*  @brief This file contains all test cases for ioHandler class
*  @copyright MIT License (c) 2020 Vasista and Vishnuu.
*/
#include <gtest/gtest.h>
#include <bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include <opencv2/tracking/tracker.hpp>
#include "robot.hpp"

/**
* @brief Test case to test the parsing of the cfg file
* @param None
* @return None
*/
TEST(ioHandling, parsing) {
  // IOHandler ioh;
  string path = "../cfg/test.cfg";
  string inFile = "../data/sample_image.png";
  string outFile = "../result/sample_img.png";
  string modelConfigPath = "../model/yolov3.cfg";
  string modelWeightsPath = "../model/yolov3.weights";
  vector<double> intrinsics = {100, 100, 194, 172.5, 2000};
  vector<double> transform = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  double humanHeight = 1.7;

  IOHandler ioh(path);
  // ioh.argParse(path);
  EXPECT_EQ(ioh.getInputType(), true);
  EXPECT_EQ(ioh.isVisualize(), false);
  EXPECT_EQ(ioh.ifRecord(), false);
  EXPECT_EQ(ioh.getHumanHeight(), humanHeight);
  EXPECT_EQ(ioh.getInFilePath(), inFile);
  EXPECT_EQ(ioh.getOutFilePath(), outFile);
  EXPECT_EQ(ioh.getModelConfigPath(), modelConfigPath);
  EXPECT_EQ(ioh.getModelWeightsPath(), modelWeightsPath);
  for (unsigned int i = 0; i< transform.size(); ++i)
    EXPECT_EQ(ioh.getTransform()[i], transform[i]);
  for (unsigned int i = 0; i< intrinsics.size(); ++i)
    EXPECT_EQ(ioh.getIntrinsics()[i], intrinsics[i]);
}

/**
 * @brief Test case to test all the utility functions such as seeImg, saveImg etc
 * @param None
 * @return None
 */
TEST(ioHandling, utilTesting) {
  string path = "../cfg/test.cfg";
  string inFile = "../data/sample_image.png";
  string outFile = "../result/sample_img.png";
  IOHandler ioh(path);

  cv::Mat inImg = cv::imread(inFile);
  vector<vector<int>> bbs = {
      {169, 43, 108, 268},
      {55, 45, 101, 287}};

  vector<cv::Scalar> colors(bbs.size(), cv::Scalar(0, 255, 0));

  ioh.drawBb(bbs, inImg, colors);
  ioh.seeImg(inImg, 1);
  ioh.saveImg(outFile, inImg);
  EXPECT_EQ(1, 1);
}
