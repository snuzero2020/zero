#include <iostream>
#include <vector>

#include "ros/ros.h"
//#include "opencv2/opencv.hpp"

#ifndef PLACE
#define PLACE

#define KCity 1
#define FMTC 2

#endif
/*
double a1 = 33.358382978338263;
double b1 = -0.043763656504531664;
double c1 = -9765560.7805824;
double a2 = -0.043763656504529812;
double b2 = -33.358382978338255;
double c2 = 138047583.23571858;
*/
/* Old FMTC map
double a1 = 33.349818513399164;
double b1 = -0.041049840938207113;
double c1 = -9774234.5960532594;
double a2 = -0.041049840938208966;
double b2 = -33.349818513399164;
double c2 = 138011335.78259039;
*/
/* New FMTC map

*/


void XYToPixel(int& pixel_x, int& pixel_y, double x, double y, bool is_kcity) {
    if (is_kcity) {
        double a1 = 33.30845062530643;
        double b1 = 0.006115022759639878;
        double c1 = -10098174.098572133;
        double a2 = 0.006115022759636954;
        double b2 = -33.30845062530642;
        double c2 = 137371264.71873185;

        pixel_x = static_cast<int>(a1*x+b1*y+c1);
        pixel_y = static_cast<int>(a2*x+b2*y+c2);
    } else {
        double a1 = 33.35682000061858;
        double b1 = -0.6892112032054343;
        double c1 = -7095938.941479745;
        double a2 = -0.6892112032054311;
        double b2 = -33.35682000061859;
        double c2 = 138233386.34684;

        pixel_x = static_cast<int>(a1*x+b1*y+c1);
        pixel_y = static_cast<int>(a2*x+b2*y+c2);
    }

    if (is_kcity) ROS_DEBUG_STREAM("XYToPixel(K-City): Coordiate (" << pixel_x << ", " << pixel_y << ")");
    else ROS_DEBUG_STREAM("XYToPixel(FMTC): Coordiate (" << pixel_x << ", " << pixel_y << ")");
}

void PixelToXY(double& x, double& y, int pixel_x, int pixel_y, bool is_kcity) {
    if (is_kcity) {
        double a1 = 0.030022410154851;
        double b1 = 5.51174605692027E-06;
        double c1 = 10098174.098572133;
        double a2 = 5.51174605691764E-06;
        double b2 = -0.030022410154851;
        double c2 = -137371264.71873185;

        pixel_x += c1; pixel_y += c2;
        x = a1*pixel_x+b1*pixel_y;
        y = a2*pixel_x+b2*pixel_y;
    } else {
        double a1 = 0.02996608409;
        double b1 = -0.000619152571;
        double c1 = 7095938.941479745;
        double a2 = -0.000619152571;
        double b2 = -0.02996608409;
        double c2 = -138233386.34684;

        pixel_x += c1; pixel_y += c2;
        x = a1*pixel_x+b1*pixel_y;
        y = a2*pixel_x+b2*pixel_y;
    }

    if (is_kcity) ROS_DEBUG_STREAM("PixelToXY(K-City): Coordiate (" << x << ", " << y << ")");
    else ROS_DEBUG_STREAM("PixelToXY(FMTC): Coordiate (" << x << ", " << y << ")");
}

// int XYToPixel_internal(cv::Mat img, double x, double y, int& pixel_x, int& pixel_y, double ref_x, double ref_y, int ref_pixel_x, int ref_pixel_y, double coefficient) {
//     //transform the coordinate into pixel

    // pixel_x = static_cast<int>((x - ref_x) / coefficient) + ref_pixel_x;
    // pixel_y = static_cast<int>(-(y - ref_y) / coefficient) + ref_pixel_y;

    // ROS_DEBUG_STREAM(std::setprecision(3) << std::fixed << "XYToPixel: before bounding - (" << x << ", " << y << ") -> (" << pixel_x << ", " << pixel_y << ")");

    // //limit the position of the car into the image

    // if (pixel_x < 10.5 / coefficient) {pixel_x = static_cast<int>(10.5 / coefficient) + 1;} // 4.5 * sqrt(5) m
//     if (pixel_x > img.cols - 10.5 / coefficient) {pixel_x = img.cols - static_cast<int>(10.5 / coefficient) - 1;}
//     if (pixel_y < 10.5 / coefficient) {pixel_y = static_cast<int>(10.5 / coefficient) + 1;}
//     if (pixel_y > img.rows - 10.5 / coefficient) {pixel_y = img.rows - static_cast<int>(10.5 / coefficient) - 1;}

//     ROS_DEBUG_STREAM(std::setprecision(3) << std::fixed << "XYToPixel: after bounding - (" << x << ", " << y << ") -> (" << pixel_x << ", " << pixel_y << ")");

//     ROS_DEBUG("XYToPixel: End--------------------------------------");

//     return 0;
// }

// int XYToPixel(cv::Mat img, double x, double y, int& pixel_x, int& pixel_y, int place) {
//     ROS_DEBUG("XYToPixel: Start--------------------------------------");
//     if (place == KCity) {
//         ROS_DEBUG("XYToPixel: The place is 'K-City'");
//         XYToPixel_internal(img, x, y, pixel_x, pixel_y, 302536.722, 4124121.856, 4090, 4127, 0.1578331);
//         return 0;
//     } else if (place == FMTC) {
//         ROS_DEBUG("XYToPixel: The place is 'FMTC'");
//         XYToPixel_internal(img, x, y, pixel_x, pixel_y, 298441.46, 4137766.57, 8864, 5268 + 60, 0.02992); //0.030066145520144317498496692
//         return 0;
//     } else {
//         return -2;
//     }
// }

/* test code
int main() {
    int pixel_x, pixel_y;
    XYToPixel(298451.28, 4137735.57, pixel_x, pixel_y, FMTC);
    std::cout << pixel_x << ", " << pixel_y << std::endl;
}
*/
