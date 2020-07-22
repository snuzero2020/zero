#include <iostream>
#include "opencv2/opencv.hpp"
#include <list>
#include "ros/ros.h"

#define _USE_MATH_DEFINES
#include <cmath>

#include "UTM.h"

using namespace std;
using namespace cv;


Mat cut_image(Mat img, Range col, Range row) {
    Mat img_cut = img(col, row);
    return img_cut;
}

int cut_map(Mat& map, double x, double y, double heading) {
    if (map.empty()) {
            cout << fixed << "cut_map: Global map is empty!" << endl;
            return -1;
    }

    //retrive a position of car

    vector<double> position_xy(2);

    position_xy[0] = x;
    position_xy[1] = y;

    cout << "cut_map: Car's position(UTM52): " << position_xy.at(0) << ", " << position_xy.at(1) << endl;
    vector<int> position_pixel(2, -1);

    //transform the coordinate into pixel

    position_pixel.at(0) = static_cast<int>((position_xy.at(0) - 302536.722) / 0.1578331) + 4090;
    position_pixel.at(1) = static_cast<int>(-(position_xy.at(1) - 4124121.856) / 0.1578331) + 4127;

    //limit the position of the car

    if (position_pixel.at(0) < 200 + 2801) {position_pixel.at(0) = 200 + 2801;}
    if (position_pixel.at(0) > map.cols - 200 - 2801) {position_pixel.at(0) = map.cols - 200 - 2801;} //302540.424,4123769.418, 302495.080,4124467.705
    if (position_pixel.at(1) < 200 + 474) {position_pixel.at(1) = 200 + 474;}
    if (position_pixel.at(1) > map.rows - 200 - 474) {position_pixel.at(1) = map.rows - 200 - 474;}

    cout << "cut_map: Car's position(px): " << position_pixel.at(0) << ", " << position_pixel.at(1) << endl;

    //rotate the image

    Mat map_rotated;

    Mat matRotation = getRotationMatrix2D(Point(position_pixel.at(0), position_pixel.at(1)), heading / M_PI * 180, 1);
    warpAffine(map, map_rotated, matRotation, map.size());

    Range range1(position_pixel.at(0) - 200, position_pixel.at(0) + 200);
    Range range2(position_pixel.at(1) - 200, position_pixel.at(1) + 200);
    cout << "cut_map: Range generated" << endl;

    if (position_pixel.at(0) < 0 || position_pixel.at(0) > map_rotated.cols || position_pixel.at(1) < 0 || position_pixel.at(1) > map_rotated.rows) {
        cout << "cut_map: Error occured!" << endl;
        return -1;
    }

    map = cut_image(map_rotated, range2, range1);
}
