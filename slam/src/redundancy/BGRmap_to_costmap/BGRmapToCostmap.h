#ifndef BGRMAPTOCOSTMAP_H
#define BGRMAPTOCOSTMAP_H

#include <iostream>
#include "ros/ros.h"

// OpenCV 3 header files
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <chrono>

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
        static int rest_sec;
        static int rest_hour;
        static double rest_time;

        static int status;
        static int bbox_up, bbox_down, bbox_left, bbox_right;

        static double getDistance(Point point1, Point point2);
        // Calculate the (Euclidian) distance between received two points.
        //
        // Input
        //   Point point1, Point point2: Two points between which calculate the distance.
        //
        // Return
        //   The Euclidian distance between received two points.
        
        static int getDistanceInt(Point point1, Point point2);
        // Calculate the (Euclidian) distance between received two points, and translate it into the intager value.
        // 
        // Input
        //   Point point1, Point point2: Two points between which calculate the distance.
        //
        // Return
        //   The integer Euclidian distance between received two points.

        static void calculateCost(int row_start, int row_end, int id);
        // Calculate a bounding box on which cost will be distributed, and distribute costs based on the static member variables(lambda function).
        //
        // Input
        //   int row_start, int row_end: Determine where calculation starts from the row, and ends.
        //   int id: ID of the thread that perform the calcutation.

    public:
        BGRmapToCostmap(Mat p_BGRmap);
        // Constructor with a received colored(BGR) map.
        // By using this constructor, you can mount the colored map you want.
        //
        // Input
        //   Mat p_BGRmap: The colored map you want.
        
        void setBGRmap(Mat p_BGRmap);
        // Function with a received colored(BGR) map.
        // If you use the constructor without any parameter, then you should this function so that mounting a colored map.
        //
        // Input
        //   Mat p_BGRmap: The colored map you want.

        Mat getCostmap();
        // Retrive the costmap from the object you created.
        //
        // Return
        //   The costmap from the object.

        void transform(function<double(uchar, uchar, uchar)>& weight, function<int(int)>& formula, int p_scope, double p_threshold, int core);    
        // Transform the BGR image as the object's member variable into the costmap image(grayscale), using the weight lambda function and foumula(e.g. 1/r^2) lambda function passed.
        //
        // Input
        //   function<double(uchar, uchar, uchar)>& weight: The weight applied to this transfomation.
        //   function<int(int)>& formula: The formula applied to this transfomation(cost distribution). This formula is like 1/r^2 or 1/arctan(r).
        //                                Assume this parameter is '1/r', for example, then a cost will be distributed propotionally '1/r'(r is distance) from the center(cost_seed).
        //   int p_scope: Determine how much far cost will be distributed.
        //                If this parameter is negative, then 'p_scope' is ignored, but 'p_threshold' become important.
        //                If this parameter is not negative, then the cost will be distributed to the distance from the center, this 'p_scope'.
        //   double p_threshold: This parameter is applied only when 'p_scope' is negative.
        //                       if 'p_threshold' < formula(distance) * 100, then minimum value of distance that satisfy the left condition become 'p_scope', and 'scope'(private member variable).
        //
        //                       * note: 100 is maximum value for the costs.
        //   int core: Determine hou much threads will be created to execute this transformation.
};

#endif