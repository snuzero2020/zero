#ifndef BGRMAPTOCOSTMAP_H
#define BGRMAPTOCOSTMAP_H

#include <iostream>
#include "ros/ros.h"

// OpenCV 3 header files
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <chrono>

#define FMTC 1
#define KCity 2

using namespace std;
using namespace cv;

class BGRmapToCostmap {
    private:
        static Mat BGRmap;
        static Mat costmap;

        static int cols;
        static int rows;
        static int pixels;

        static int scope;
        static int threshold;

        static uchar* BGRmap_data;
        static uchar* costmap_data;

        static function<double(uchar, uchar, uchar)> weight;
        static function<double(double)> formula;

        static std::chrono::system_clock::time_point timeStart;

        static vector<int> thread_ids;

        static int elipsed_min;
        static int elipsed_sec;
        static int elipsed_msec;

        static int rest_min;
        static float rest_sec;
        static int rest_hour;
        static double rest_time;

        static int status;
        static int bbox_up, bbox_down, bbox_left, bbox_right;

        static double getDistance(Point point1, Point point2);
        static int getDistanceInt(Point point1, Point point2);
        static void calculateCost(int row_start, int row_end, int id);

    public:
        BGRmapToCostmap(Mat p_BGRmap);
        void setBGRmap(Mat p_BGRmap);
        Mat getCostmap();

        void transform(function<double(uchar, uchar, uchar)>& weight, function<double(double)>& formula, int p_scope, double p_threshold, int core);    
};

#endif