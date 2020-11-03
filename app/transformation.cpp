/**
 *  @file    transformation.cpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief This file has all function definitions for Transformation class
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu.
 */
#include <bits/stdc++.h>
#include <Eigen/Dense>
#include "transformation.hpp"

using vision::Transformation;

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
	delete this->transform;
}

/**
* @brief assigns the Transformation matrix from the camera frame to the robot frame
* @param transMat - the transformation matrix from the camera frame to the robot frame
* @return None
*/
void Transformation::setTransform(Eigen::Matrix4f transMat) {
	this->transform = new Eigen::Matrix4f;
	*(this->transform) = transMat;
    return;
}

/**
* @brief return the Transformation matrix from the camera frame to the robot frame
* @param None
* @return transMat - the transformation matrix from the camera frame to the robot frame
*/
Eigen::Matrix4f Transformation::getTransform() {
    Eigen::Matrix4f transformationMat = *(this->transform);
    return transformationMat;
}

/**
* @brief Transforms a 3D coordinate in the camera frame to the robot frame
* @param camCord - 3D coordinate in the camera frame
* @return newCords - New cordinates in the robot frame
*/
Eigen::Vector4f Transformation::transformToRoboFrame(Eigen::Vector4f camCord) {
    Eigen::Vector4f newCords  = (*(this->transform))*camCord;
    return newCords;
}
