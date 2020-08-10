#include "MapCutter.hpp"

#include <iostream>
#include "opencv2/opencv.hpp"
#include "ros/ros.h"
#include <string>
#include <vector>

#define _USE_MATH_DEFINES
#include <cmath>

#include "XYToPixel.h"
#include "UTM.h"

#ifndef PLACE
#define PLACE

#define KCity 1
#define FMTC 2

#endif

using namespace std;
using namespace cv;

Mat MapCutter::FMTC_map = Mat::zeros(1, 1, CV_8U);
vector<Mat> MapCutter::KCity_maps = {Mat::zeros(1, 1, CV_8U)};

void MapCutter::loadMap() {
    bool exist_error = false;

    ROS_INFO("MapCutter: Loading FMTC and K-City maps...");

    KCity_maps.clear();
    
    FMTC_map = imread("/home/dongha/catkin_ws/src/zero/slam/src/mapping/costmap.png", IMREAD_GRAYSCALE);
    if (FMTC_map.empty()) {
            ROS_ERROR("MapCutter: The FMTC cost map is empty");
            exist_error = true;
    } else {
        ROS_INFO("MapCutter: The FMTC cost map was loaded");
    }

    KCity_maps.clear();

    for (int n = 0; n < 12; n++) {
        string path = "map_KCity_" + to_string(n + 1) + ".png";
        KCity_maps.push_back(imread(path));
        
        if (KCity_maps[n].empty()) {
            exist_error = true;

            switch ((n + 1) % 20) {
                case 1:
                    ROS_ERROR("MapCutter: The %dst K-City map is empty", n + 1);
                    break;

                case 2:
                    ROS_ERROR("MapCutter: The %dnd K-City map is empty", n + 1);
                    break;

                case 3:
                    ROS_ERROR("MapCutter: The %drd K-City map is empty", n + 1);
                    break;

                default:
                    ROS_ERROR("MapCutter: The %dth K-City map is empty", n + 1);
                    break;
            }
        } else {
            switch ((n + 1) % 20) {
                case 1:
                        ROS_INFO("MapCutter: The %dst K-City map was loaded", n + 1);
                        break;

                case 2:
                    ROS_INFO("MapCutter: The %dnd K-City map was loaded", n + 1);
                    break;

                case 3:
                    ROS_INFO("MapCutter: The %drd K-City map was loaded", n + 1);
                    break;

                default:
                    ROS_INFO("MapCutter: The %dth K-City map was loaded", n + 1);
                    break;
            }
        }
    }
    ROS_ERROR_COND(exist_error, "MapCutter: Could not load the one or more maps! Check whether the image file(s) exist, OR rosrun is executed at ~/catkin_ws");
    ROS_INFO_COND(!exist_error, "MapCutter: All needed maps(FMTC, K-City) were successfully loaded!");
}


MapCutter::MapCutter() {
    MapCutter(KCity);
    ROS_WARN("MapCutter: No received place, assume the place is 'K-City'");
}

MapCutter::MapCutter(int place) {
    if (place != FMTC && place != KCity) {
        this->place = KCity;

        ROS_WARN("MapCutter: Wrong received place, assume the place is 'K-City'");
    }

    else if (place == FMTC) {
        this->place = place;

        number_of_maps = 1;
        ROS_INFO("MapCutter: The place is 'FMTC'");
    }

    else if (place == KCity) {
        this->place = place;

        number_of_maps = 12;
    }

    position_px.push_back(0);
    position_px.push_back(0);
}

int MapCutter::cutViaPxCenter(Mat& original_map, Mat& modified_map, int pixel_x, int pixel_y) {
    Range range_row(pixel_x - 337, pixel_x + 337);
    Range range_col(pixel_y - 337, pixel_y + 337);

    modified_map = original_map(range_col, range_row);

    ROS_DEBUG("MapCutter: cut => cutViaPxCenter");
    return 0;

}

Mat MapCutter::smartCut(double x, double y, double heading) {
    Mat modified_map;

    if (place == FMTC) {
        ROS_DEBUG("MapCutter: smartCut => cut");
        cut(FMTC_map, modified_map, x, y, heading);
        ROS_DEBUG("MapCutter: cut => smartCut");
        
    }

    return modified_map;
}

int MapCutter::cut(Mat& original_map, Mat& modified_map, double x, double y, double heading) {
    XYToPixel(original_map, x, y, position_px[0], position_px[1], place);

    // pre-cutting
    Mat cut_map;
    ROS_DEBUG_STREAM("MapCutter: " << position_px[0] << ", " << position_px[1]);

    if (position_px[0] < 337) {position_px[0] = 337;}
    if (position_px[0] > original_map.cols - 337) {position_px[0] = original_map.cols - 337;}

    if (position_px[1] < 337) {position_px[1] = 337;}
    if (position_px[1] > original_map.cols - 337) {position_px[1] = original_map.cols - 337;}

    cutViaPxCenter(original_map, cut_map, position_px[0], position_px[1]);
    ROS_DEBUG("MapCutter: cutViaPxCenter => smartCut");

    //rotate the image
    Mat rotated_map;

    Mat matRotation = getRotationMatrix2D(Point(337, 337), heading * 180 / M_PI , 1);
    warpAffine(cut_map, rotated_map, matRotation, cut_map.size());

    // main cutting
    Range range_row(337 - 150, 337 + 150);
    Range range_col(337 - 300, 337);

    modified_map = rotated_map(range_col, range_row);

    return 0;
}

