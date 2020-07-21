#include "opencv2/opencv.hpp"

#define KCity 1
#define FMTC 2

int XYToPixel(cv::Mat img, double x, double y, int& pixel_x, int& pixel_y, int place);