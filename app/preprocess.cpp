/**
 *  @file    preprocess.cpp
 *  @author  Vasista (clueless-bachu)
 *  @author  Vishnuu (vishnuu95)
 *  @brief This file has all function definitions for Preprocessor class
 *  @copyright MIT License (c) 2020 Vasista and Vishnuu.
 */
#include <bits/stdc++.h>
#include "opencv2/opencv.hpp"
#include "preprocess.hpp"

using vision::Preprocessor;

/**
* @brief applies various prprocessing steps to raw images to condition them for detection
* and tracking. This includes resizing, gaussian blur, 
* @param size - The final size of the preprocessed image
* @param blurKernel - Kernel size for gaussian blur
* @param inImg - the pointer to the image that needs to be processed
* @param inImg - the output pointer to the image where the final processed image is written
* @return None
*/
void Preprocessor::preprocess
(pair<int, int> size, int blurKernel, cv::Mat* inImg, cv::Mat* outImg) {

	cv::resize(*inImg, *outImg, cv::Size(size.first, size.second), 0, 0, cv::INTER_LINEAR);

	if(blurKernel)
	cv::GaussianBlur(*outImg, *outImg, cv::Size(blurKernel, blurKernel), 0);

    return;
}
