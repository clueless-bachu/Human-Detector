/**
 *  @file    detTrack.hpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @date    10/20/2020
 *  @brief Detection and Tracking Module decleration.
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu
 */
#pragma once
#include <string>
#include <vector>

using std::string;
using std::vector;
namespace vision {

class DetTrack {
 private:
  cv::Ptr<cv::MultiTracker> trackers;
  vector<vector<int>> trackersBbox;
  cv::dnn::Net yoloModel;
  vector<string> outNames;
  int imageWidth;
  int imageHeight;
  vector<cv::Mat> detectedObjects;
  float confThreshold;
  float nmsThreshold;
  int count = 0;

 public:
    DetTrack(string, string);
    ~DetTrack();
    void addTrackers(cv::Mat *, vector<vector<int>>);
    vector<vector<int>> detectHumans(cv::Mat *);
    vector<vector<int>> trackHumans(cv::Mat *);
};

}  // namespace vision
