#include <gtest/gtest.h>
#include <bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include <opencv2/tracking/tracker.hpp>
#include <Eigen/Dense>
#include "robot.hpp"

/*
(169, 43, 108, 268) -> right human
(55, 45, 101, 287) -> left human
Intrinsic matrix 
K = [
	100, 0, 0
	0, 100, 0
	0, 0 ,0
	] in mm

Average height = 1.70
s = 2000 pixels/mm
*/

TEST(depthEstimation, estimateDepth) {
  DepthEstimator depthE;
  vector<vector<int>> bbs = {
  	{169, 43, 108, 268},
  	{55, 45, 101, 287}
  };

  vector<double> depths = {1.26, 1.18};
  vector<double> computedDepths = depthE.estimateDepth(bbs);
  double threshold = 0.5;
  EXPECT_EQ(computedDepths.size(), depths.size());
  ASSERT_NEAR(depths[0], computedDepths[0], threshold);
  ASSERT_NEAR(depths[1], computedDepths[1], threshold);
}


TEST(depthEstimation, estimate) {
  DepthEstimator depthE;
  double threshold = 0.5;
  vector<vector<int>> bbs = {
  	{169, 43, 108, 268},
  	{55, 45, 101, 287}
  };

  vector<vector<double>> positions = {
  	{0.35,0.1,1.26},
  	 {-0.49,0.0345,1.18}
  };
  vector<vector<double>> computedPositions = depthE.transform2dTo3d(bbs);

  EXPECT_EQ(computedPositions.size(), positions.size());
  ASSERT_NEAR(computedPositions[0][0], computedPositions[0][0], threshold);
  ASSERT_NEAR(computedPositions[0][1], computedPositions[0][1], threshold);
  ASSERT_NEAR(computedPositions[0][2], computedPositions[0][2], threshold);
  ASSERT_NEAR(computedPositions[1][0], computedPositions[1][0], threshold);
  ASSERT_NEAR(computedPositions[1][1], computedPositions[1][1], threshold);
  ASSERT_NEAR(computedPositions[1][2], computedPositions[1][2], threshold);
}