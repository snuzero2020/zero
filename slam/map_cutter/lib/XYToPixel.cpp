#include <iostream>
#include <vector>
#include "opencv2/opencv.hpp"

#define KCity 1
#define FMTC 2

int XYToPixel_internal(cv::Mat img, double x, double y, int& pixel_x, int& pixel_y, double ref_x, double ref_y, int ref_pixel_x, int ref_pixel_y, double coefficient) {
    //transform the coordinate into pixel

    pixel_x = static_cast<int>((x - ref_x) / coefficient) + ref_pixel_x;
    pixel_y = static_cast<int>(-(y - ref_y) / coefficient) + ref_pixel_y;

    //limit the position of the car into the image

    if (pixel_x < 14.5 / coefficient) {pixel_x = static_cast<int>(14.5 / coefficient) + 1;} // 10 * sqrt2 m
    if (pixel_x > img.cols - 14.5 / coefficient) {pixel_x = img.cols - static_cast<int>(14.5 / coefficient) - 1;}
    if (pixel_y < 14.5 / coefficient) {pixel_y = static_cast<int>(14.5 / coefficient) + 1;}
    if (pixel_y > img.rows - 14.5 / coefficient) {pixel_y = img.rows - static_cast<int>(14.5 / coefficient) - 1;}

    return 0;
}

int XYToPixel(cv::Mat img, double x, double y, int& pixel_x, int& pixel_y, int place) {
    if (place == KCity) {
        XYToPixel_internal(img, x, y, pixel_x, pixel_y, 302536.722, 4124121.856, 4090, 4127, 0.1578331);
        return 0;
    } else if (place == FMTC) {
        XYToPixel_internal(img, x, y, pixel_x, pixel_y, 298441.46, 4137766.57, 6890, 1707, 0.03);
        return 0;
    } else {
        return -2;
    }
}

/* test code
int main() {
    int pixel_x, pixel_y;
    XYToPixel(298451.28, 4137735.57, pixel_x, pixel_y, FMTC);
    std::cout << pixel_x << ", " << pixel_y << std::endl;
}
*/