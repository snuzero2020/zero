#include <iostream>
#include "ros/ros.h"

// OpenCV 3 header files
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "BGRmapToCostmap.h"

#include <cmath>

#define FMTC 1
#define KCity 2

using namespace std;
using namespace cv;

int main() {
    Mat map(imread("src/zero/slam/src/BGRmap_to_costmap/FMTC.png", IMREAD_COLOR));

    if (map.empty()) {
        ROS_ERROR("The loaded map is empty");
        return 0;
    }
    BGRmapToCostmap translator(map);

    function<double(uchar, uchar, uchar)> weight([](uchar B, uchar G, uchar R) {
        if (R != 0 && B == 0) {return R * (100.0 / 255.0);}
        else {return static_cast<double>((255 - B) * (100.0 / 255.0));}
    });

    function<double(double)> formula([](double distance) {
        if (distance < 1) {return 1.0;}
        else {return (1.0 / pow(distance, 2.0));} // 1/r^2
    });

    translator.transform(weight, formula, -1, 1, 8);

    Mat costmap = translator.getCostmap();
    imwrite("src/zero/slam/src/BGRmap_to_costmap/FMTC_cost.png", costmap);
}