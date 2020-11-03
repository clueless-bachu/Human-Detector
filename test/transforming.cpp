/**
 *  @file    transforming.cpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief  Test file that tests functionalities of Transformation class
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu.
 */
#include <gtest/gtest.h>
#include <bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include <opencv2/tracking/tracker.hpp>
#include "transformation.hpp"

using vision::Transformation;
using std::vector;

/**
 * @brief Tests the Initialisation of the class
 * @param None
 * @return None
 */
TEST(trans, initialisation) {
    Transformation transformer;
    vector<double> cam2roboTrans;
    cam2roboTrans = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    transformer.setTransform(cam2roboTrans);
    EXPECT_EQ(transformer.getTransform(), cam2roboTrans);
}

/**
 * @brief Tests the Transformation of a 4-vector
 * @param None
 * @return None
 */
TEST(transformation, transformCords) {
    Transformation transformer;
    vector<double> cam2roboTrans;
    cam2roboTrans = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
    transformer.setTransform(cam2roboTrans);

    vector<double> camCords = {1, 2, 3};
    EXPECT_EQ(transformer.transformToRoboFrame(camCords), camCords);
}
