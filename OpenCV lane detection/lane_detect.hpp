#pragma once
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include "IPM.h"
#include <iostream>
#include <vector>
#include <sstream>

class lane_detect {
public:
	lane_detect();
	lane_detect(int xSigmai, int ySigmai, int upperThreshi, int lowerThreshi, int minLineLengthi, int maxLineGapi);

	void detect();
	void show_windows(cv::Mat hough_image, cv::Mat canny_image, cv::Mat gaussian_image, cv::Mat original);
	void applyHSV(cv::Mat orig, cv::Mat output, int iLowH, int iHighH, int iLowS, int iHighS, int iLowV, int iHighV);
	cv::Mat gaussian_blur(cv::Mat image);
	cv::Mat canny_edge(cv::Mat gaussian_image);
	cv::Mat sobel_edge(cv::Mat gaussian_image);
	cv::Mat hough_transform(cv::Mat canny_image, cv::Mat original);

	IPM init_ipm(int width, int height, std::vector<cv::Point2f>& origPoints, std::vector<cv::Point2f>& dstPoints);

private:
	int xSigma;
	int ySigma;

	int upperThresh;
	int lowerThresh;

	int minLineLength;
	int maxLineGap;

	cv::Mat gaussian_image;
	cv::Mat canny_image;
	cv::Mat original;
	cv::Mat hough_image;
};
