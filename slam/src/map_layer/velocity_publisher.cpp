#include <iostream>
#include <ros/ros.h>
#include "ros/package.h"
#include "opencv2/opencv.hpp"
#include <vector>
#include <slam/Pixel.h>
#include <slam/Data.h>
#include "XYToPixel.h"
#include "std_msgs/Float32.h"
#include <opencv2/core/persistence.hpp>
#include <fstream>
#include <string>

class velocity_publisher{

    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        int pixel_x, pixel_y;
        bool x_inRange{(pixel_x<=14500)}, y_inRange{(pixel_y<=14500)};
        float recommended_velocity;


    public:
        std::stringstream path_stream;
        cv::Mat velocity_map;
       
        velocity_publisher(){
        ROS_INFO("0");
	    path_stream << ros::package::getPath("slam") << "/config/test.xml";
        ROS_INFO("1");
        cv::FileStorage fs;
        fs.open(path_stream.str(), cv::FileStorage::READ); 
        if(!fs.isOpened())
        {
            std::cout<<"Failed to open"<<std::endl;
        }
        ROS_INFO("2");
        fs["velocity_map"] >> velocity_map;
        ROS_INFO("Image loaded");
        cv::FileNode n = fs.root();
        for(cv::FileNodeIterator current = n.begin(); current !=n.end(); current++){
            cv::FileNode item = *current;
            cv::Mat v;
            item["test"] >>v;
            std::cout<<"1"<<std::endl;
            std::cout<<v<<std::endl;
        }
        std::cout<<velocity_map<<std::endl;

        pub = nh.advertise<std_msgs::Float32>("/recommended_velocity", 2);
        sub = nh.subscribe("/filtered_data",2, &velocity_publisher::callback, this);
        }

        void callback(const slam::Data::ConstPtr& msg){
         XYToPixel(pixel_y, pixel_x, msg->x, msg->y);
         std::cout<<"Pixel information is loaded: "<<pixel_x<<", "<<pixel_y<<std::endl;

         if(x_inRange&&y_inRange){
             std::cout<<"on map"<<std::endl;
             //publish the information of a pixel
             recommended_velocity = velocity_map.at<float>(pixel_x, pixel_y);
             std_msgs::Float32 rt;
             rt.data = recommended_velocity;
             pub.publish(rt);
         }
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "velocity_publisher");
    velocity_publisher velocity_publisher;
    ros::spin();
}

