#include "ros/ros.h"
#include <iostream>
#include "lanenet_lane_detection/lanenet_clus_msg.h"
#include <opencv2/opencv.hpp>
#include <ctime>
#include <vector>
#include "multi_img_utils.h"
#include <cmath>
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"
#include "cv_bridge/cv_bridge.h"

using namespace cv;
using namespace std;

class LanenetWindow{
    private:
        ros::NodeHandle nh;
        ros::Subscriber lanenet_sub;
        ros::Publisher costMap_pub;
    
    public:
        LanenetWindow(){
            costMap_pub = nh.advertise<sensor_msgs::Image>("/lanenet_costMap", 1);
            //lanenet_sub = nh.subscribe("/lane_seg_topic", 1, &LanenetWindow::lanenet_callback, this);
            ROS_INFO("LanenetWindow loaded");
        }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lanenet_seg_subscriber");
    
    ros::spin();
    return 0;
}