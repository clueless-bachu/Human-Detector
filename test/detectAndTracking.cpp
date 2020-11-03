/**
 *  @file    main.cpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief This file contains test all test cases for Detection and Tracking
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu.
 */
#include <bits/stdc++.h>
#include <gtest/gtest.h>
#include <gmock/gmock.h>
#include "opencv2/opencv.hpp"
#include <opencv2/tracking/tracker.hpp>
#include "robot.hpp"

using std::max;
using std::min;
using std::string;
using std::vector;

/**
* @brief Class for unit testing the DetTrack Class
* @param None
* @return None
*/
class checkDetTrackClass : public ::testing::Test {
 protected:  // default variables
  string modelConfigPath = "";
  string modelWeightsPath = "";
  string imgPath = "";
  string labelPath = "";
  cv::Mat img;  // = cv::imread(imgPath);
  DetTrahck *tracker;

 public:
  /**
  * @brief Constructor class for checkDetTrackClass
  * @param None
  * @return None
  */h
  checkDetTrackClass() {
  }  // constructor

  /**
  * @brief Destructor class for checkDetTrackClass
  * @param None
  * @return None
  */
  virtual ~checkDetTrackClass() {
  }  // destructor

  /**
  * @brief required method that is called at the start of every TEST_F
  * @param None
  * @return None
  */
  virtual void SetUp() {
    string modelConfigPath = "../model/yolov3.cfg";
    string modelWeightsPath = "../model/yolov3.weights";
    imgPath = "/home/vasista/Desktop/Human-Detector/data/sample_image.png";
    tracker = new DetTrack(modelConfigPath, modelWeightsPath);
  }

  /**
  * @brief required method that is called at the end of every TEST_F. It deletes the DetTrack 
  * object
  * @param None
  * @return None
  */
  virtual void TearDown() {
    delete tracker;
  }

  /**
  * @brief calculates the IoU metric for two bounding boxes
  * @param boxA - the first bounding box
  * @param boxB - the second bounding box
  * @return iou - the metric value of IoU
  */
  double calcIOU(vector<int> boxA, vector<int> boxB) {
    auto xA = max(boxA[0], boxB[0]);
    auto yA = max(boxA[1], boxB[1]);
    auto xB = min(boxA[2], boxB[2]);
    auto yB = min(boxA[3], boxB[3]);
    //  compute the area of intersection rectangle
    double interArea = max(0, xB - xA + 1) * max(0, yB - yA + 1);
    // # compute the area of both the prediction and ground-truth
    // # rectangles
    auto boxAArea = (boxA[2] - boxA[0] + 1) * (boxA[3] - boxA[1] + 1);
    auto boxBArea = (boxB[2] - boxB[0] + 1) * (boxB[3] - boxB[1] + 1);
    // # compute the intersection over union by taking the intersection
    // # area and dividing it by the sum of prediction + ground-truth
    // # areas - the interesection area
    double iou = interArea
    / static_cast<double>(boxAArea + boxBArea - interArea);
    // # return the intersection over union value
    return iou;
  }
};

/**
 * @brief Test case for testing the detector and tracker
 * @param None
 * @return None
*/
TEST_F(checkDetTrackClass, detectAndTrackTest) {
  img = cv::imread(imgPath);
  vector<vector<int>> labels(2, vector<int>(5, 1));
  auto iouThreshold = 0.7;
  labels[1][0] = 169;
  labels[1][1] = 43;
  labels[1][2] = 108+169;
  labels[1][3] = 268+43;
  labels[1][4] = 1;
  labels[0][0] = 55;
  labels[0][1] = 45;
  labels[0][2] = 101+55;
  labels[0][3] = 287+45;
  labels[0][4] = 2;
  cv::Mat pImg;
  cv::resize(img, pImg, cv::Size(416, 416));
  vector<vector<int>> boxes = tracker->trackHumans(&pImg);

  bool match_found = false;
  for (auto box : boxes) {
    match_found = false;
    for (auto label : labels) {
      vector<int> bb = {box[0], box[1], box[0]+box[2], box[1]+box[3]};
      auto iou = calcIOU(bb, label);
      if (iou >= iouThreshold) {
        match_found = true;
        break;
      }
    }
    ASSERT_TRUE(match_found);
  }
  ASSERT_NE(boxes[0][4], boxes[1][4]);
}
