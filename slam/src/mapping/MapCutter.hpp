#ifndef MAPCUTTER_H
#define MAPCUTTER_H

#include <iostream>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <string>
#include <vector>

#define FMTC 2
#define KCity 1

using namespace std;
using namespace cv;


class MapCutter {
    private:
        int number_of_maps;
        vector<double> position_xy;
        vector<int> position_px;
        int place;
    public:
        MapCutter();
        MapCutter(int place);
	vector<Mat> map_list;
        int cutViaPxCenter(Mat& original_map, Mat& modified_map, int pixel_x, int pixel_y);
        Mat smartCut(double lat, double lon, double heading);
        int cut(Mat& original_map, Mat& modified_map, double lat, double lon, double heading);
};

#endif
