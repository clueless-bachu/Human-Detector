/**
 *  @file    depthEstimator.cpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief This file has all function definitions for DepthEstimator class
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu.
 */
#include <bits/stdc++.h>
#include "depthEstimator.hpp"

using vision::DepthEstimator;
using std::vector;

/**
* @brief A constructor function for the DepthEstimator class
* @param None
* @return None
*/
DepthEstimator::DepthEstimator() {
    // this->humanHeight = humanHeight;
    // this->intrinsicParams = intrinsicParams;
}

/**
* @brief A destructor function for the DepthEstimator class
* @param None
* @return None
*/
DepthEstimator::~DepthEstimator() {
}

/**
* @brief Estimates the depths for a set of humans represented by a vector of bounding boxes
* @param bbs - a vector of bounding boxes where each bounding box is a vector of int 
* representing a human
* @return depths - a vector of doubles representing the depth values of each bounding box
*/
vector<double> DepthEstimator::estimateDepth
(vector<vector<int>> bbs, double humanHeight, vector<double> intrinsicParams) {
    vector<double> depths(bbs.size());

    for (unsigned int i = 0; i< bbs.size(); ++i) {
        depths[i] = intrinsicParams[0]*0.001*
        humanHeight*intrinsicParams[4]/bbs[i][3];
    }
    return depths;
}

/**
* @brief takes image cordinates of each human and computes the 3D cordinates of the human
* in camera coordinate system
* @param bbs - a vector of bounding boxes where each bounding box is a vector of int 
* representing a human
* @return depths - a vector of cordinates representing the x,y,z values with
* respect to the camera coordinate frame
*/
vector<vector<double>> DepthEstimator::transform2dTo3d
(vector<vector<int>> bbs, double humanHeight, vector<double> intrinsicParams) {
    vector<vector<double>> cords3d(bbs.size(), vector<double>(bbs.size()));

    vector<double> depths = this->estimateDepth(bbs, humanHeight,
                                  intrinsicParams);

    for (unsigned int i = 0; i< bbs.size(); ++i) {
        double x, y, X, Y;
        x = (bbs[i][0]+bbs[i][2])/2;
        y = (bbs[i][1]+bbs[i][3])/2;

        X = (x - intrinsicParams[2])*depths[i]/
                  (intrinsicParams[1]*0.001*intrinsicParams[4]);
        Y = (y - intrinsicParams[3])*depths[i]/
                  (intrinsicParams[2]*0.001*intrinsicParams[4]);
        vector<double> pos = {X, Y, depths[i]};
        cords3d[i] = pos;
    }

    return cords3d;
}

