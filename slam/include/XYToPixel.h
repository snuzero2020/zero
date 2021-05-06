#include "opencv2/opencv.hpp"

#ifndef PLACE
#define PLACE

#define KCity 1
#define FMTC 2
#define FMTC_cost 3

#endif

/**
@brief UTM52N 좌표(x, y)를 지도 픽셀 값(pixel_x, pixel_y)로 변환한다.

변환된 픽셀 값이 실제 지도의 범위를 벗어나는지는 고려하지 않는다(= 유효성 검사를 하지 않는다).

@param pixel_x 픽셀 x 좌표를 받아올 변수.
@param pixel_y 픽셀 y 좌표를 받아올 변수.
@param x 변환할 UTM52N x 좌표.
@param y 변환할 UTM52N y 좌표.
@param is_kcity 장소. true는 K-City를, false는 FMTC를 나타낸다.
*/
void XYToPixel(int& pixel_x, int& pixel_y, double x, double y, bool is_kcity);
/**
@brief 지도 픽셀 값(pixel_x, pixel_y)을 UTM52N 좌표(x, y)로 변환한다.

@remark 변환된 좌표 값이 실제 지도의 범위를 벗어나는지는 고려하지 않는다. 유효성 검사는 직접 해야 한다.

@param x UTM52N x 좌표를 받아올 변수.
@param y UTM52N y 좌표를 받아올 변수.
@param pixel_x 변환할 픽셀 x 좌표.
@param pixel_y 변환할 픽셀 y 좌표.
@param is_kcity 장소. true는 K-City를, false는 FMTC를 나타낸다.
*/
void PixelToXY(double& x, double& y, int pixel_x, int pixel_y, bool is_kcity);

//int XYToPixel(cv::Mat img, double x, double y, int& pixel_x, int& pixel_y, int place);
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
