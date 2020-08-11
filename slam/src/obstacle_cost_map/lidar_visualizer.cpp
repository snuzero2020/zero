#include "ros/ros.h"
#include "ros/time.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Eigen/Eigen"
#include <vector>
//Message Type
#include "slam/imgCluster.h"
#include "geometry_msgs/Point.h"

using namespace cv;
using namespace std;

class Visualizer{
    public:
    Visualizer(){
        sub_ = nh_.subscribe("/pcl_on_image", 1, &Calibration::callback, this);
    }

    void callback(){

    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};


int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_visualizer");
    Visualizer visualizer;
    ros::spin();
}