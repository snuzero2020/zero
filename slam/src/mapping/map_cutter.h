#ifndef MAPCUTTER_H
#define MAPCUTTER_H

#include <iostream>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <string>
#include <vector>

#define FMTC 1
#define KCity 2

using namespace std;
using namespace cv;


class MapCutter {
    private:
        int number_of_maps;
        vector<Mat> map_list();
        vector<double> position_xy(2);
        vector<int> position_px(2);
        int place;
    public:
        MapCutter();
        MapCutter(int place);

        int cutViaPxCenter(Mat& original_map, Mat& modified_map, int pixel_x, int pixel_y);
        Mat smartCut(double lat, double lon, double heading);
        int cut(Mat& original_map, Mat& modified_map, double lat, double lon, double heading);
};

#endif