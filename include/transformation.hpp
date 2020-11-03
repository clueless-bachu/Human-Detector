/**
 *  @file    transformation.hpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief Transformation module that transforms Human coordinates from Camera frame to Robot Frame. 
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu.
 */
#pragma once
#include<bits/stdc++.h>
#include<vector>

using std::vector;

namespace vision {

class Transformation {
 private:
    std::vector<double> transform;

 public:
    Transformation();
    ~Transformation();
    void setTransform(std::vector<double>);
    std::vector<double> getTransform();
    std::vector<double> transformToRoboFrame(std::vector<double>);
};

}  // namespace vision
