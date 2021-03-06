/**
 *  @file    robot.cpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief This file has all function definitions for Robot class
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu.
 */
#include <bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include <opencv2/tracking/tracker.hpp>
#include "robot.hpp"
cv::RNG rng(12344);
using robot::Robot;

/**
* @brief A constructor function for the Transformation class
* @param modelPath:string - path to the config file
* @return None
*/
Robot::Robot(string cfgPath) {
  ioh = new IOHandler(cfgPath);
  humanDetector = new DetTrack
    (ioh->getModelConfigPath(), ioh->getModelWeightsPath());
  depthEstimator = new DepthEstimator();
  transformer = new Transformation();
}

/**
* @brief A destructor function for the Robot class
* @param None
* @return None
*/
Robot::~Robot() {
  delete ioh;
  delete humanDetector;
    delete depthEstimator;
    delete transformer;
}

/**
* @brief this reads from the video/image, detect all humans using YOLO and tracks them using 
* a MultiTracker. Then it draws the bounding boxes and writes the 3D cordinates onto the 
* image, displays and saves it in the output file
* @param modelPath:string - path to the DNN model file
* @return None
*/
void Robot::processData() {
  vector<cv::Scalar> colors;
  for (int i = 0; i < 100; ++i) {
    colors.emplace_back(cv::Scalar(rng.uniform(0, 255),
     rng.uniform(0, 255), rng.uniform(0, 255)));
  }

  if (ioh->getInputType()) {
        cv::Mat frame;
        frame = cv::imread(ioh->getInFilePath());
        //     preprocess img
        std::cout<< "Reading image" << std::endl;
        cv::Mat pFrame;
        Preprocessor::preprocess(std::pair<int, int>
          (ioh->yoloWidth, ioh->yoloHeight), 3, &frame, &pFrame);
        //     track humans
        vector<vector<int>> bboxes = humanDetector->trackHumans(&pFrame);
        ioh->drawBb(bboxes, pFrame, colors);  // needs to be changed
        // ioh save img
        if (ioh->isVisualize()) {
            ioh->seeImg(pFrame, 2000);
        }
        if (ioh->ifRecord()) ioh->saveImg(ioh->getOutFilePath(), pFrame);

        vector<double> trans = ioh->getTransform();

        transformer->setTransform(trans);
        vector<vector<double>> positionsCam = depthEstimator->
        transform2dTo3d(bboxes, ioh->getHumanHeight(), ioh->getIntrinsics());

        std::stringstream ss;
        for (unsigned int i = 0; i < positionsCam.size(); ++i) {
          vector<double> pos =
           transformer->transformToRoboFrame(positionsCam[i]);
          ss << "***** 3D position of Box " << i+1 << " *****" << std::endl
          <<pos[0] << std::endl <<pos[1] << std::endl << pos[2] << std::endl;
        }

        std::cout<< ss.str() << std::endl;

  } else {
      cv::VideoCapture cap(ioh->getInFilePath());
      if (!cap.isOpened()) {
          std::cout << "Error opening video stream or file" << std::endl;
          return;
      }

      cv::VideoWriter video;
      if (ioh->ifRecord()) {
          video = cv::VideoWriter(ioh->getOutFilePath(),
          cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 60,
          cv::Size(416, 416));
      }


      vector<double> trans = ioh->getTransform();

      transformer->setTransform(trans);
      std::stringstream ss;
      int count = 1;
      while (1) {
          cv::Mat frame;
          cap >> frame;
          if (frame.empty())
            break;

          cv::Mat pFrame;
          Preprocessor::preprocess(std::pair<int, int>
            (ioh->yoloWidth, ioh->yoloHeight), 3, &frame, &pFrame);
          vector<vector<int>> bboxes = humanDetector->trackHumans(&pFrame);
          ioh->drawBb(bboxes, pFrame, colors);

          if (ioh->isVisualize()) cv::imshow("Frame", pFrame );
          if (ioh->ifRecord()) video.write(pFrame);

          vector<vector<double>> positionsCam = depthEstimator->
          transform2dTo3d(bboxes, ioh->getHumanHeight(), ioh->getIntrinsics());

          for (unsigned int i = 0; i< positionsCam.size(); ++i) {
            vector<double> pos =
             transformer->transformToRoboFrame(positionsCam[i]);
            ss << "***** 3D position of Box " << i+1 << " *****" << std::endl
            <<pos[0] << std::endl <<pos[1] << std::endl << pos[2] << std::endl;
          }
          ss<< "----------------End of Frame---------------\n";
          char c = static_cast<char>(cv::waitKey(1));
          if (c == 27)
            break;
          ++count;
        }
      cap.release();

      if (ioh->isVisualize()) cv::destroyAllWindows();
      if (ioh->ifRecord()) video.release();
      std::cout<< ss.str()<< std::endl;
  }
  return;
}
