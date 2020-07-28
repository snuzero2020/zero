#include "opencv2/opencv.hpp"

#ifndef PLACE
#define PLACE

#define KCity 1
#define FMTC 2
#define FMTC_cost 3

#endif

int XYToPixel(cv::Mat img, double x, double y, int& pixel_x, int& pixel_y, int place);
// Convert a UTM52 coordinate to a position(pixel) on the received map.
// 
// Input
//     Mat img: The map on which the car drive.
//     double x, double y: The car's position in UTM52 coordinate(x, y).
//     int& pixel_x, int& pixel_y: The car's converted position(pixel_x, pixel_y).
//     int place:
//         1: K-City
//         2: FMTC
//
// Return
//     0: No error.
//    -1: One or more errors occured. 
