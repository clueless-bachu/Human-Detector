/**
 *  @file    robot.cpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief  Test file that tests functionalities of robot class
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu.
 */
#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include <opencv2/tracking/tracker.hpp>
#include "robot.hpp"

using robot::Robot;

/**
 * @brief Test case to test the initialisation of the robot with the test config file
 * @param None
 * @return None
 */
TEST(robotTest, dataProcessing) {
  string cfgPath = "../cfg/test.cfg";
  Robot robot(cfgPath);
  robot.processData();
  EXPECT_EQ(1, 1);
}
