/**
 *  @file    depthEstimator.hpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @date    10/20/2020
 *  @brief Depth Estimation module definition under vision namespace. 
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu
 */
#pragma once
#include <vector>

using std::vector;

namespace vision {

class DepthEstimator {
 private:
 public:
    DepthEstimator();
    ~DepthEstimator();
    vector<double> estimateDepth(vector<vector<int>>, double, vector<double>);
    vector<vector<double>>
     transform2dTo3d(vector<vector<int>>, double, vector<double>);
};

}  // namespace vision
