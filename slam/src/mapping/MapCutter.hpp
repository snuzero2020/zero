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

        int cutViaPxCenter(Mat& original_map, Mat& modified_map, int pixel_x, int pixel_y);
        // Cut the received image into 675 X 675, getting (pixel_x, pixel_y) to the center, (338, 338).
        // Cut map is returned to the parameter 'Modified_map'.
        //
        // Return
        //     0: No error.
        //    -1: One or more errors occured. 
        //
        // Note: 337 [px] = 4.5 * sqrt(5) [m] (note below)

        int cut(Mat& original_map, Mat& modified_map, double x, double y, double heading);
        // Rotate by the parameter 'heading', and cut the receive image properly.
        // The car's position is like below.
        //
        // Return
        //     0: No error.
        //    -1: One or more errors occured. 
        //
        // |--- 4.5 m ---|--- 4.5 m ---|
        // ----------------------------- ---
        // |             |             |  |
        // |             |             |  |
        // |             |             |  |
        // |             |             |  
        // |             |             | 9.0 m
        // |             |             | 
        // |             |             |  |
        // |             |             |  |
        // |             ^             |  |
        // --------------O-------------- ---
        //               ^ car's position (driving forward)
        //
        //              
        // --------------O-------------- ---
        // |             ^ car's position (driving backword)
        // |             |             |  |
        // |             |             |  |
        // |             |             |  
        // |             |             | 9.0 m
        // |             |             | 
        // |             |             |  |
        // |             |             |  |
        // |             |             |  |
        // ----------------------------- ---
        // |--- 4.5 m ---|--- 4.5 m ---|             
        
    public:
        static Mat FMTC_map;
        static vector<Mat> KCity_maps;

        MapCutter();
        // Constructor without place for driving.
        // => Assume the place is K-City.
        
        MapCutter(int place);
        // Constructor with place to drive.
        //
        // Input
        //     int place:
        //         1: K-City
        //         2: FMTC
        
        static void loadMap();
        // Load the map files for K-City and FMTC in advance for immediate processing.
        // Users should execute the function before subscribing a coordinate and a heading, in other words, creating a MapCutter object.

        Mat smartCut(double x, double y, double heading);
        // Cut the map properly, by the received car's position and heading.
        //
        // Input
        //     double x, double y:
        //         The car's position (x, y) in UTM52.
        //     double heading:
        //         The car's heading(radian). Counterclockwise is positive, and initial line is y-axis.
        //
        // Return
        //     The cut map.
};

#endif
