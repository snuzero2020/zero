#include "func_util.h"
#include "lane_fit_sliding_window.h"
#include "lane_class.h"
#include <ctime>

int main(void)
{
	int time = clock();
	cv::Mat img;
	img = cv::imread("test_images/test2.jpg");

	if(img.empty())
	{
		std::cerr << "Image load failed!" << std::endl;
		return -1;
	}

	cv::Mat perspective_img;
	perspective_img = birdeye(img);

//	cv::namedWindow("perspective_img");
//	cv::imshow("perspective_img",perspective_img);

	cv::Mat yellow;
	yellow = thresh_frame_in_HSV(perspective_img);
//	cv::namedWindow("yellow_img");
//	cv::imshow("yellow_img",yellow);

	cv::Mat grad;
	grad = thresh_frame_sobel(perspective_img);
//	cv::namedWindow("grad_img");
//	cv::imshow("grad_img", grad);	

	cv::Mat gray;
	gray = get_binary_from_equalized_grayscale(perspective_img);
//	cv::namedWindow("gray_img");
//	cv::imshow("gray_img", gray);	

	cv::Mat lane_mask;
	lane_mask = get_lane_mask(yellow, grad, gray);
//	cv::namedWindow("lane_mask");
//	cv::imshow("lane_mask", lane_mask);

	cv::Mat fitting_mask;
	fitting_mask = get_fits_by_sliding_window(lane_mask, 10);
	//cv::namedWindow("fitting_mask");
	//cv::imshow("fitting_mask", fitting_mask);

	std::cout << (clock() - time)/(double)CLOCKS_PER_SEC << std::endl;

	cv::waitKey();
	return 0;
}
