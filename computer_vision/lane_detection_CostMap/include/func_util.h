#ifndef FUNC_UTIL_H
#define FUNC_UTIL_H

#include <iostream>
#include "opencv2/opencv.hpp"

cv::Mat birdeye(cv::Mat img);
cv::Mat thresh_frame_sobel(cv::Mat src);
cv::Mat get_binary_from_equalized_grayscale(cv::Mat src);
cv::Mat get_lane_mask(cv::Mat yellow, cv::Mat grad, cv::Mat gray);
cv::Mat thresh_frame_in_HSV(cv::Mat src);

#endif
