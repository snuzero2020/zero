#include <iostream>
#include <ros/ros.h>
#include "ros/package.h"
#include "opencv2/opencv.hpp"
#include <vector>
#include <slam/Pixel.h>
#include <slam/Data.h>
#include "XYToPixel.h"
#include "std_msgs/Float64.h"
//#include <opencv2/core/persistence.hpp>
//#include <fstream>
#include <string>

class velocity_publisher{

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        int pixel_x, pixel_y;
        bool x_inRange{(pixel_x<=14500)}, y_inRange{(pixel_y<=14500)};
        double recommended_velocity;


    public:
        std::stringstream path_stream;
        //cv::Mat velocity_map;
        cv::Mat velocity_map = cv::imread("/home/healthykim/catkin_ws/src/zero/slam/config/KCity/KCity_velocity_map2.png");

        velocity_publisher(){
	    //path_stream << ros::package::getPath("slam") << "/config/FMTC/velocity_map.png";
        //cv::Mat velocity_map = cv::imread(path_stream.str());
        ROS_INFO("Image loaded");

        pub = nh.advertise<std_msgs::Float64>("/recommended_velocity", 2);
        sub = nh.subscribe("/filtered_data",2, &velocity_publisher::callback, this);
        }

        void callback(const slam::Data::ConstPtr& msg){
         XYToPixel(pixel_y, pixel_x, msg->x, msg->y);
         std::cout<<"Pixel information is loaded: "<<pixel_x<<", "<<pixel_y<<std::endl;

         if(x_inRange&&y_inRange){
             std::cout<<"on map"<<std::endl;
             //publish the information of a pixel
             recommended_velocity = velocity_map.at<cv::Vec3b>(pixel_x, pixel_y)[0];
             std_msgs::Float64 rt;
             rt.data = recommended_velocity/85;
             pub.publish(rt);
         }
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "velocity_publisher");
    velocity_publisher velocity_publisher;
    ros::spin();
}

