/**
 *  @file    estimatingDepth.cpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief This file contains all test cases for Depth Estimation module
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu.
 */
#include <Eigen/Dense>
#include <gtest/gtest.h>
#include <bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include <opencv2/tracking/tracker.hpp>
#include "robot.hpp"

/**
 * @brief Test case to estimate depth value of detected objects
 * @param None
 * @return None
 */
TEST(depthEstimation, estimateDepth) {
  DepthEstimator depthE;
  vector<vector<int>> bbs = {
      {169, 43, 108, 268},
      {55, 45, 101, 287}};
  double humanHeight = 1.7;
  vector<double> intrinsicParams = {100, 100, 194, 172.5, 2000};

  vector<double> depths = {1.26, 1.18};
  vector<double> computedDepths = depthE.estimateDepth(bbs, humanHeight, intrinsicParams);
  double threshold = 0.5;
  EXPECT_EQ(computedDepths.size(), depths.size());
  ASSERT_NEAR(depths[0], computedDepths[0], threshold);
  ASSERT_NEAR(depths[1], computedDepths[1], threshold);
}

/**
 * @brief Test case to estimate the 2d to 3d transfromation of coordinates
 * @param None
 * @return None
 */
TEST(depthEstimation, estimatePos) {
  DepthEstimator depthE;
  double threshold = 0.5;
  vector<vector<int>> bbs = {
      {169, 43, 108, 268},
      {55, 45, 101, 287}};
  double humanHeight = 1.7;
  vector<double> intrinsicParams = {100, 100, 194, 172.5, 2000};

  vector<vector<double>> positions = {
      {0.35, 0.1, 1.26},
      {-0.49, 0.0345, 1.18}};
  vector<vector<double>> computedPositions = depthE.transform2dTo3d(bbs, humanHeight, intrinsicParams);

  EXPECT_EQ(computedPositions.size(), positions.size());
  ASSERT_NEAR(computedPositions[0][0], computedPositions[0][0], threshold);
  ASSERT_NEAR(computedPositions[0][1], computedPositions[0][1], threshold);
  ASSERT_NEAR(computedPositions[0][2], computedPositions[0][2], threshold);
  ASSERT_NEAR(computedPositions[1][0], computedPositions[1][0], threshold);
  ASSERT_NEAR(computedPositions[1][1], computedPositions[1][1], threshold);
  ASSERT_NEAR(computedPositions[1][2], computedPositions[1][2], threshold);
}
