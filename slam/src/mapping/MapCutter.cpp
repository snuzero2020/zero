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

#define KCity 1
#define FMTC 2

using namespace std;
using namespace cv;

Mat MapCutter::FMTC_map = Mat::zeros(1, 1, CV_8UC3);
vector<Mat> MapCutter::KCity_maps = {Mat::zeros(1, 1, CV_8UC3)};

void MapCutter::loadMap() {
    bool exist_error = false;

    FMTC_map = imread("src/zero/slam/src/mapping/map.png", IMREAD_COLOR);
    if (FMTC_map.empty()) {
            ROS_ERROR("MapCutter: The FMTC map is empty");
            exist_error = true;
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
        }
    }
    if (exist_error == true) {
        cout << "27[1;31m" <<"MapCutter: => Check whether the image file exist, OR rosrun is executed at ~/catkin_ws" << "27[0m" << endl;
    }   
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
    Range range_x(pixel_x - 337, pixel_x + 337);
    Range range_y(pixel_y - 337, pixel_y + 337);

    modified_map = original_map(range_x, range_y); // 333 [px] = 10 [m] / 0.03 [m/px]

    ROS_INFO("MapCutter: success cutting a received map");
    return 0;

}

Mat MapCutter::smartCut(double x, double y, double heading) {
    Mat modified_map;

    if (place == FMTC) {
        cut(FMTC_map, modified_map, x, y, heading);
    }

    return modified_map;
}

int MapCutter::cut(Mat& original_map, Mat& modified_map, double x, double y, double heading) {
    XYToPixel(original_map, x, y, position_px[0], position_px[1], place);

    // pre-cutting
    Mat cut_map;
    cout << position_px[0] << ", " << position_px[1] << endl;

    if (position_px[0] < 300) {position_px[0] = 300;}
    if (position_px[0] > original_map.cols - 300) {position_px[0] = original_map.cols - 300;}

    if (position_px[1] < 300) {position_px[1] = 300;}
    if (position_px[1] > original_map.cols - 300) {position_px[1] = original_map.cols - 300;}

    cutViaPxCenter(original_map, cut_map, position_px[0], position_px[1]);

    //rotate the image
    Mat rotated_map;

    Mat matRotation = getRotationMatrix2D(Point(337, 337), heading / M_PI * 180, 1);
    warpAffine(cut_map, rotated_map, matRotation, cut_map.size());

    // main cutting
    Range range1(337 - 150, 337 + 150);
    Range range2(337 - 300, 337);

    modified_map = rotated_map(range1, range2);

    return 0;
}

