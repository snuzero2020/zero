#include <iostream>
#include <ros/ros.h>
#include "ros/package.h"
#include "opencv2/opencv.hpp"
#include <vector>
#include <slam/Pixel.h>
#include "std_msgs/UInt32.h"


class velocity_publisher{

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        int pixel_x, pixel_y;
        bool x_inRange{(pixel_x<=14500)}, y_inRange{(pixel_y<=14500)};
        int recommend_velocity;


    public:
        std::stringstream path_stream;
    	cv::Mat velocity_map;
        velocity_publisher(){
            path_stream << ros::package::getPath("slam") << "/config/velocity_map.png";
		    cv::Mat color_map = cv::imread(path_stream.str());
            ROS_INFO("Image loaded");
            pub = nh.advertise<std_msgs::UInt32>("/recommended_velocity", 2);
            sub = nh.subscribe("/filtered_data",2, &sector_publisher::callback, this);
        }

        void callback(const slam::Data::ConstPtr& msg){
         XYToPixel(pixel_y, pixel_x, msg->x, msg->y);
         std::cout<<"Pixel information is loaded: "<<pixel_x<<", "<<pixel_y<<std::endl;

         if(x_inRange&&y_inRange){
             std::cout<<"on map"<<std::endl;
             //publish the information of a pixel
             recommended_velocity = velocity_map.at<cv::Vec3b>(pixel_x, pixel_y)[0];
             std_msgs::UInt32 rt;
             rt = recommended_velocity;
             pub.publish(rt);
         }
        }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "velocity_publisher");
    velocty_publisher velocity_publisher;
    ros::spin();
}
