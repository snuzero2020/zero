#include "func_util.h"
#include <iostream>
#include "opencv2/opencv.hpp"

cv::Mat birdeye(cv::Mat img)
{
	int w = img.cols;
	int h = img.rows;

	int out_w = 200;
	int out_h = 200;

	cv::Point2f pt1(w, h-10);
	cv::Point2f pt2(0, h-10);
	cv::Point2f pt3(546, 460);
	cv::Point2f pt4(732, 460);

	cv::Point2f src_vertices[4];
	src_vertices[0] = pt1;
	src_vertices[1] = pt2;
	src_vertices[2] = pt3;
	src_vertices[3] = pt4;

	cv::Point2f dst_vertices[4];
	dst_vertices[0] = cv::Point2f(out_w, out_h);
	dst_vertices[1] = cv::Point2f(0,out_h);
	dst_vertices[2] = cv::Point2f(0,0);
	dst_vertices[3] = cv::Point2f(out_w,0);

	cv::Mat M = cv::getPerspectiveTransform(src_vertices, dst_vertices);
	cv::Mat img_out;

	warpPerspective(img, img_out, M, cv::Size(out_w,out_h));
	return img_out;
}

cv::Mat thresh_frame_in_HSV(cv::Mat src)
{
	cv::Mat hsv = cv::Mat::zeros(src.size(), CV_8UC3);
	cv::Mat yellow = cv::Mat::zeros(src.size(), CV_8UC1);
	cv::cvtColor(src, hsv, cv::COLOR_BGR2HSV_FULL);
	cv::inRange(src, cv::Scalar(0, 50, 100), cv::Scalar(124, 255, 255), yellow);
	return yellow;
}

cv::Mat thresh_frame_sobel(cv::Mat src)
{
	int scale = 1;
	int delta = 0;

	cv::Mat gray = cv::Mat::zeros(src.size(), CV_8UC1);
	cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

	cv::Mat grad;
	cv::Mat grad_x, grad_y;
	cv::Mat abs_grad_x, abs_grad_y;

	cv::Sobel(gray, grad_x, CV_16S, 1, 0, 3, scale, delta, cv::BORDER_DEFAULT);
	cv::Sobel(gray, grad_y, CV_16S, 0, 1, 3, scale, delta, cv::BORDER_DEFAULT);

	cv::convertScaleAbs(grad_x, abs_grad_x);
	cv::convertScaleAbs(grad_y, abs_grad_y);
	cv::addWeighted(abs_grad_x, 1, abs_grad_y, 1, 0, grad);

	return grad;
}

cv::Mat get_binary_from_equalized_grayscale(cv::Mat src)
{
	cv::Mat gray;
	cv::cvtColor(src, gray, cv::COLOR_BGR2GRAY);

	cv::Mat gray_equalize;
	cv::equalizeHist(gray, gray_equalize);

	cv::Mat gray_out;
	cv::threshold(gray_equalize, gray_out, 250, 255, cv::THRESH_BINARY);

	return gray_out;
}

cv::Mat get_lane_mask(cv::Mat yellow, cv::Mat grad, cv::Mat gray)
{
	int img_size = 200;
	cv::Mat lane_mask = cv::Mat::zeros(img_size, img_size, CV_8UC1);

	for(int i=0; i< img_size; i++)
	{
		for(int j=0; j< img_size; j++)
		{
			if(grad.at<uchar>(i,j) == 255 && gray.at<uchar>(i,j) == 255)
			{
				lane_mask.at<uchar>(i,j) = 128;
			}
			else if(grad.at<uchar>(i,j) == 255 && yellow.at<uchar>(i,j) == 255)
			{
				lane_mask.at<uchar>(i,j) = 255;
			}
			else
			{
				lane_mask.at<uchar>(i,j) = 0;
			}
		}
	}	

	return lane_mask;
}

