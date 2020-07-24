#ifndef MAPCUTTER_H
#define MAPCUTTER_H

#include <iostream>
#include "opencv2/core.hpp"
#include "ros/ros.h"
#include <string>
#include <vector>

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

        static Mat FMTC_map;
        static vector<Mat> KCity_maps;

        static void loadMap();
        int cutViaPxCenter(Mat& original_map, Mat& modified_map, int pixel_x, int pixel_y);
        Mat smartCut(double lat, double lon, double heading);
        int cut(Mat& original_map, Mat& modified_map, double lat, double lon, double heading);
};

#endif
