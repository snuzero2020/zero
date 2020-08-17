#include <iostream>
#include <string.h>
#include <vector>

#include "geometry_msgs/Point.h"

#include "slam/Clustermaster.h"
#include "slam/Clusters.h"
#include "slam/imgCluster.h"

#include "ros/ros.h"
#include "ros/time.h"
#include <opencv2/opencv.hpp>
#include "Eigen/Eigen"

using namespace std;
using namespace cv;


class Movement{
    public:
    Movement(){

    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
};


int main(int argc, char **argv){
    ros::init(argc, argv, "move_lidar");
    Movement movement_;
    ros::spin();
}