#include <iostream>
#include "ros/ros.h"

// OpenCV 3 header files
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include "BGRmapToCostmap.h"

#include <cmath>

#ifndef PLACE
#define PLACE

#define KCity 1
#define FMTC 2

#endif

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
        else {return (255 - B) * (50.0 / 255.0);}
    });

    //cout << weight(0,0,0) << endl;

    function<int(int)> formula([](int distance) {
        if (distance == 0) {return 1;}
        else {return (3 / (distance + 2));} // 1/r
    });

    translator.transform(weight, formula, 87, 1, 30);

    Mat costmap = translator.getCostmap();
    imwrite("src/zero/slam/src/BGRmap_to_costmap/FMTC_cost.png", costmap);
}
