/**
* @file detTrack.cpp
* @authors Phase 1 - Vasista Ayyagari (Driver), Vishnuu Appaya Dhanabalan (Navigator)
* @brief This file has all function definitions for DetTrack class
* @copyright Vasista Ayyagari, Vishnuu Appaya Dhanabalan, 2020
*/
#include <bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include <opencv2/tracking/tracker.hpp>
#include <opencv2/dnn/dnn.hpp>
#include "detTrack.hpp"
#include <typeinfo>
#include <opencv2/core/utility.hpp>
#include <opencv2/tracking.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <cstring>
#include <ctime>

using std::vector;
using vision::DetTrack;
using namespace cv;

/**
* @brief A constructor function for the DetTrack class
* @param configPath:string - path to the DNN model file
* @return None
*/
DetTrack::DetTrack(string configPath, string weightsPath) {
    yoloModel = cv::dnn::readNetFromDarknet(configPath, weightsPath);
    outNames = yoloModel.getUnconnectedOutLayersNames();
    imageWidth = 416;
    imageHeight = 416;
    confThreshold = 0.5;
    nmsThreshold = 0.7;
    yoloModel.setPreferableBackend(cv::dnn::DNN_BACKEND_DEFAULT);
    yoloModel.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    trackers = cv::MultiTracker::create();
}

/**
* @brief A destructor function for the DetTrack class
* @param None
* @return None
*/
DetTrack::~DetTrack() {
}

/**
* @brief checks if new humans have entered the scene and assigns each new human a tracker
* @param bbDet - Bounding boxes obtained by running detection using YOLO
* @param bbTrack - Bounding boxes obtained from the MultiTracker on the new frame
* @return None
*/
void DetTrack::addTrackers
(cv::Mat *inImg, vector<vector<int>> bbDet) {
    std::vector<Ptr<Tracker> > algorithms;
    vector<Rect2d> objects;
    for(auto box : bbDet){
        if(box[3]-box[1]< 100) continue;
        algorithms.push_back(cv::TrackerCSRT::create());
        objects.push_back(cv::Rect(box[0], box[1], box[2]-box[0], box[3]-box[1]));
    }

    trackers->add(algorithms,*inImg,objects);
}


/**
* @brief runs rYOLO on a processed input image and detects all the humans
* @param inImg - input image which needs to have th humans detected
* @return None
*/
vector<vector<int>> DetTrack::detectHumans(cv::Mat *inImg) {
    Mat blob = cv::dnn::blobFromImage(*inImg, 1/255.0, Size(imageWidth, imageHeight), Scalar(), false, false);
    yoloModel.setInput(blob, ""); // remaining params were optional.
    yoloModel.forward(detectedObjects, outNames);
    // std::cout << detectedObjects.size() << std::endl;
    // std::cout << detectedObjects[1].size() << std::endl;
    // std::cout << typeid(detectedObjects[0]).name() << std::endl;
    vector<vector<int>> bbs;
    vector<int> classes;
    vector<float> confidenceVals;
    vector<cv::Rect> predictions;
    if(detectedObjects.size() > 0) {
        // iterate over detections
        for(auto detections : detectedObjects) {
            float* detection = reinterpret_cast<float *>(detections.data);
            for(auto i = 0; i < detections.rows; i++, detection += detections.cols) {
                cv::Mat predScores = detections.row(i).colRange(5, detections.cols);
                cv::Point2i classId;
                double confidence;
                cv::minMaxLoc(predScores, 0, &confidence, 0, &classId);
                // std::cout << "confidence vals"<< confidence << std::endl;
                if(confidence > confThreshold) {
                    auto centerXCoord = static_cast<int>(detection[0]*(inImg->cols));
                    auto centerYCoord = static_cast<int>(detection[1]*(inImg->rows));
                    auto boxWidth = static_cast<int>(detection[2]*(inImg->cols));
                    auto boxHeight = static_cast<int>(detection[3]*(inImg->rows));
                    auto topLeftX = (centerXCoord - boxWidth >=0)?(centerXCoord - boxWidth):0;
                    auto topLeftY = (centerYCoord - boxHeight >=0)?(centerYCoord - boxHeight):0;
                    classes.push_back(classId.x);
                    confidenceVals.push_back(static_cast<float>(confidence));
                    predictions.push_back(cv::Rect(topLeftX, topLeftY, boxWidth, boxHeight));
                }
            }
        }
    }
    // NMSing
    vector<int> indexes;
    cv::dnn::NMSBoxes(predictions, confidenceVals, confThreshold, nmsThreshold, indexes);
    for( auto idx : indexes){
        if(classes[idx] ==0){
            // std::cout << "drawing" << std::endl;
            cv::Rect rect = predictions[idx];
            int bottomRightX = (rect.x + rect.width <=416)?(rect.x + rect.width):416;
            int bottomRightY = (rect.y + rect.height <=416)?(rect.y + rect.height):416;
            // cv::rectangle(reImg, cv::Point(rect.x, rect.y),  
            //     cv::Point(bottomRightX, bottomRightY), cv::Scalar(0,255,0), 3);
            std::vector<int> temp{rect.x, rect.y, bottomRightX, bottomRightY};
            bbs.push_back(temp);
        }
    }
    // cv::imwrite("window.bmp", reImg);
    // *inImg = reImg;
    
    return bbs;
}

/**
* @brief detects all humans using YOLO and tracks them using the MultiTracker
* @param inImg - input image which needs to have th humans detected and tracked
* @return None
*/
vector<vector<int>> DetTrack::trackHumans(cv::Mat *inImg) {
    // as of now boxes are being drawn on 416, 416 img. 
    // as of now tracking is only done on first frame detections. 
    if (count == 0) {
        auto boxes = detectHumans(inImg);
        if(boxes.size() > 0){
        addTrackers(inImg, boxes); 
        }
    }
    ++count;

    trackers->update(*inImg);
    vector<vector<int>> bbs;
    for(unsigned i=0; i< trackers->getObjects().size(); i++) {

        vector<int> bb = {  
                            (int)(trackers->getObjects()[i].x + 0.4*trackers->getObjects()[i].width),
                            (int)(trackers->getObjects()[i].y + 0.1*trackers->getObjects()[i].height),
                            (int)(trackers->getObjects()[i].width),
                            (int)(trackers->getObjects()[i].height)
                          };

        bbs.emplace_back(bb);
    }    
    
    return bbs;
}


