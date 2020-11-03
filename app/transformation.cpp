/**
 *  @file    transformation.cpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief This file has all function definitions for Transformation class
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu.
 */
#include <bits/stdc++.h>
#include "transformation.hpp"

using vision::Transformation;
using std::vector;

/**
* @brief A constructor function for the Transformation class
* @param None
* @return None
*/
Transformation::Transformation() {
}

/**
* @brief A destructor function for the Transformation class
* @param None
* @return None
*/
Transformation::~Transformation() {
}

/**
* @brief assigns the Transformation matrix from the camera frame to the robot frame
* @param transMat - the transformation matrix from the camera frame to the robot frame
* @return None
*/
void Transformation::setTransform(vector<double> transMat) {
	this->transform = transMat;
    return;
}

/**
* @brief return the Transformation matrix from the camera frame to the robot frame
* @param None
* @return transMat - the transformation matrix from the camera frame to the robot frame
*/
vector<double> Transformation::getTransform() {
    return this->transform;
}

/**
* @brief Transforms a 3D coordinate in the camera frame to the robot frame
* @param camCord - 3D coordinate in the camera frame
* @return newCords - New cordinates in the robot frame
*/
vector<double> Transformation::transformToRoboFrame(vector<double> camCord) {
    vector<double>  newCords = {
this->transform[0]*camCord[0]+this->transform[1]*camCord[1]+this->transform[2]*camCord[2]+this->transform[3],
this->transform[4]*camCord[0]+this->transform[5]*camCord[1]+this->transform[6]*camCord[2]+this->transform[7],
this->transform[8]*camCord[0]+this->transform[9]*camCord[1]+this->transform[10]*camCord[2]+this->transform[11]
    };
    return newCords;
}
