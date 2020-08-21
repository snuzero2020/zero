#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

using namespace std;
using namespace cv;


class ErodePng{
    public:
    ErodePng(){
        path_stream_ << ros::package::getPath("slam")<<"/config/FMTC/FMTC_road_area.png";
        road_map_ = imread(path_stream_.str());
        element = getStructuringElement(MORPH_RECT, Size(3,3),Point(1,1));
        erodeimage();
    }

    void erodeimage(){
        cout << road_map_.size() << endl;
        erode(road_map_, eroded, element, Point(-1,-1),7);
        eroded_stream << ros::package::getPath("slam")<<"/config/FMTC/FMTC_road_area_eroded.png";
        imwrite(eroded_stream.str(), eroded);
    }
    private:
    ros::NodeHandle nh_;
    stringstream path_stream_;
    stringstream eroded_stream;
    Mat road_map_;
    Mat element;
    Mat eroded;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "erode_png");
    ErodePng erode_png;
    return 0;
}