#include "map_cutter.h"

#include <iostream>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <string>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include "XYToPixel.h"
#include "UTM.h"

#define FMTC 1
#define KCity 2

using namespace std;
using namespace cv;


MapCutter::MapCutter() {
    MapCutter(KCity);
    ROS_WARN("MapCutter: No received place, assume the place is 'K-City'");
}

MapCutter::MapCutter(int place) {
    if (place != FMTC || place != KCity) {
        this->place = KCity;

        ROS_WARN("MapCutter: Wrong received place, assume the place is 'K-City'");
    }

    if (place == FMTC) {
        this->place = place;

        number_of_maps = 1;
        map_list.push_back(imread("map_FMTC_1.png"));
        ROS_INFO("MapCutter: The place is 'FMTC'");
    }

    string path;

    if (place == KCity) {
        this->place = place;

        number_of_maps = 12;

        for (int n = 0; n < number_of_maps; n++) {
            path = "map_KCity_" + to_string(n + 1) + ".png";
            map_list.push_back(imread(path));
            ROS_INFO("MapCutter: The place is 'K-City'");
        }
    }
}

int MapCutter::cutViaPxCenter(Mat& original_map, Mat& modified_map, int pixel_x, int pixel_y) {
    Range range_x(pixel_x - 333, pixel_x + 333);
    Range range_y(pixel_y - 333, pixel_y + 333);
    modified_map = original_map(range_x, range_y); // 333 [px] = 10 [m] / 0.03 [m/px]

    ROS_INFO("MapCutter: success cutting a received map");
    return 0;

}

Mat MapCutter::smartCut(double lat, double lon, double heading) {
    Mat modified_map;

    if (place == FMTC) {
        cut(map_list[0], modified_map, lat, lon, heading);  
    }

    return modified_map;
}

int MapCutter::cut(Mat& original_map, Mat& modified_map, double lat, double lon, double heading) {
    LatLonToUTMXY(lat, lon, 52, position_xy[0], position_xy[1]);
    XYToPixel(original_map, position_xy[0], position_xy[1], position_px[0], position_px[1], place);

    // pre-cutting
    Mat cut_map;
    cutViaPxCenter(original_map, cut_map, position_px[0], position_px[1]);

    //rotate the image
    Mat rotated_map;

    Mat matRotation = getRotationMatrix2D(Point(position_px[0], position_px[1])), heading / M_PI * 180, 1);
    warpAffine(cut_map, rotated_map, matRotation, cut_map.size());

    // main cutting
    Range range1(position_px[0] - 333, position_px[0] + 333);
    Range range2(position_px[1] - 666, position_px[1]);

    if (position_px[0] < 0 || position_px[0] > rotated_map.cols || position_px[1] < 0 || position_px[1] > rotated_map.rows) {
        ROS_ERROR("MapCutter: Cannot fit lat/lon to rotated img!");
        return -1;
    }

    modified_map = rotated_map(range1, range2);

    return 0;
}

