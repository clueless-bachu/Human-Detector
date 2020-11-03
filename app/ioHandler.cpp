/**
 *  @file    ioHandler.cpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief This file has all function definitions for IOHandler class
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu.
 */
#include <bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include <opencv2/tracking/tracker.hpp>
#include "ioHandler.hpp"

using vision::IOHandler;

/**
* @brief A constructor function for the IOHandler class
* @param None
* @return None
*/
IOHandler::IOHandler(string cfgPath) {
	this->argParse(cfgPath);
}

/**
* @brief A destructor function for the IOHandler class
* @param None
* @return None
*/
IOHandler::~IOHandler() {
}

/**
* @brief parses the config file and assigns all its attributes to the appropriate value
* @param cfgPath - A path to the cfg file 
* @return None
*/
void IOHandler::argParse(string cfgPath) {
	std::ifstream cFile (cfgPath);
    if (cFile.is_open())
    {
        std::string line;
        while(getline(cFile, line)){
            line.erase(std::remove_if(line.begin(), line.end(), isspace),
                                 line.end());
            if(line[0] == '#' || line.empty())
                continue;
            auto delimiterPos = line.find("=");
            string name = line.substr(0, delimiterPos);
            string value = line.substr(delimiterPos + 1);


            if(!name.compare("inPath")) this->inPath.assign(value);// = value.copy();
            else if(!name.compare("outPath")) this->outPath = value;
            else if(!name.compare("modelConfigPath")) this->modelConfigPath = value;
            else if(!name.compare("modelWeightsPath")) this->modelWeightsPath = value;
            else if(!name.compare("isImg") && !value.compare("true")) this->isImg = true;
            else if(!name.compare("ifVisualize") && !value.compare("true")) this->ifVisualize = true;
            else if(!name.compare("record") && !value.compare("true")) this->record = true;
            else if(!name.compare("humanHeight")) this->humanHeight = std::stod(value);
            else if(!name.compare("K")) {
            	auto newPos = value.find(",");
            	while(newPos != std::string::npos) {
            		
            		this->intrinsicParams.push_back(std::stod(value.substr(0, newPos)));
            		value = value.substr(newPos+1);
            		newPos = value.find(",");
            	}

            	this->intrinsicParams.push_back(std::stod(value));
            }
            else if(!name.compare("transform")) {
                auto newPos = value.find(",");
                while(newPos != std::string::npos) {
                    
                    this->transform.push_back(std::stod(value.substr(0, newPos)));
                    value = value.substr(newPos+1);
                    newPos = value.find(",");
                }

                this->transform.push_back(std::stod(value));
            }
            else if(!name.compare("imgWidth")) this->imgWidth = std::stoi(value);
            else if(!name.compare("imgHeight")) this->imgHeight = std::stoi(value);
        }
        
    }
    else {
        std::cerr << "Couldn't open config file for reading.\n";
    }
    return;
}

/**
* @brief returns the input type: either image or video
* @param None
* @return a boolean, if true then type is image, if false then type is video
*/
bool IOHandler::getInputType() {
    return this->isImg;
}

/**
* @brief returns condition to visualize data or not
* @param cfgPath - A path to the cfg file 
* @return None
*/
bool IOHandler::isVisualize() {
    return this->ifVisualize;
}

bool IOHandler::ifRecord() {
    return this->record;
}
/**
* @brief returns the input image/video path
* @param None
* @return path - path to the data file
*/
string IOHandler::getInFilePath() {
    return this->inPath;
}

/**
* @brief returns the image/video path where the output data should be stored
* @param None
* @return path - path to the data file
*/
string IOHandler::getOutFilePath() {
    return this->outPath;
}

/**
* @brief returns the config file path of the DNN
* @param None
* @return path - path to the data file
*/
string IOHandler::getModelConfigPath() {
    return this->modelConfigPath;
}

/**
* @brief returns the weightsfile path of the DNN
* @param None
* @return path - path to the data file
*/
string IOHandler::getModelWeightsPath() {
    return this->modelWeightsPath;
}

/**
* @brief returns the average assumed human height
* @param None
* @return human height
*/
double IOHandler::getHumanHeight() {
	return this->humanHeight;
}

/**
* @brief returns the camera intrinsics
* @param None
* @return Camera intrinsics
*/
vector<double> IOHandler::getIntrinsics() {
	return this->intrinsicParams;
}

/**
* @brief returns the 4x4 matrix camera to robot transform as a flat vector
* @param None
* @return Camera intrinsics
*/
vector<double> IOHandler::getTransform() {
    return this->transform;
}

/**
* @brief returns the image width
* @param None
* @return image width
*/
int IOHandler::getImgWidth() {
	return this->imgWidth;
}

/**
* @brief returns the image height
* @param None
* @return image height
*/
int IOHandler::getImgHeight() {
	return this->imgHeight;
}

/**
* @brief draws the bounding boxes over an image
* @param bb - a set of bounding boxes that needs to be drawn
* @param frame - the image on which the bounding boxes need to be drawn on
* @return None
*/
void IOHandler::drawBb(vector<vector<int>> bbs, cv::Mat frame, vector<cv::Scalar> colors) {

	for (unsigned int i=0; i< bbs.size(); ++i) {
		cv::Rect rect(bbs[i][0], bbs[i][1], bbs[i][2], bbs[i][3]);
		cv::rectangle(frame,rect,colors[i],3);	
	}

    return;
}

/**
* @brief visualizes an image
* @param frame - the image on which the bounding boxes need to be drawn on
* @return None
*/
void IOHandler::seeImg(cv::Mat frame, int wait) {
	cv::imshow("Human Tracking", frame);
	cv::waitKey(wait);
    return;
}

/**
* @brief save the image to a particular path
* @param path - The path where the image should be saved
* @param frame - the image to be saved
* @return None
*/
void IOHandler::saveImg(string path, cv::Mat frame) {
	cv::imwrite(path, frame);
    return;
}
