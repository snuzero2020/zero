#include <iostream>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <vector>
#include <slam/Pixel.h>
#include "std_msgs/UInt32.h"


class sector_publisher{
    public:
        cv::Mat color_map = cv::imread("/home/healthykim/catkin_ws/src/zero/slam/src/mapping/color_map.png");
        sector_publisher(){
            pub = nh.advertise<std_msgs::UInt32>("/sector_info", 1000);
            sub = nh.subscribe("/position/pixel", 1000, &sector_publisher::callback, this);
            nBlue = 0; nGreen = 0; nRed = 0;
        }

        void callback(const slam::Pixel Data){
         pixel_x = Data.x;
         pixel_y = Data.y;
         std::cout<<"Pixel information is loaded: "<<pixel_x<<", "<<pixel_y<<x_inRange<<y_inRange<<std::endl;

         if(x_inRange&&y_inRange){
             std::cout<<"on map"<<std::endl;
             //get the RGB information of a pixel 
             nBlue = color_map.at<cv::Vec3b>(pixel_x, pixel_y)[0];
             nGreen = color_map.at<cv::Vec3b>(pixel_x,pixel_y)[1];
             nRed = color_map.at<cv::Vec3b>(pixel_x,pixel_y)[2];
             
             //publish
             // in sector A ==> pub 0
             //           B ==> pub 1
             //           C ==> pub 2
             //           D ==> pub 3

             std_msgs::UInt32 rt;
             if(nBlue==0&&nGreen==0&&nRed==0)
             {
                 rt.data = 0;
                 ROS_INFO("Sector A");
             }
             if(nBlue==0&&nGreen==255&&nRed==0)
             {
                 rt.data = 1;
                 ROS_INFO("Sector B");
             }
             if(nBlue==255&&nGreen==0&&nRed==0)
             {
                 rt.data = 2;
                 ROS_INFO("Sector C");
             }
             if(nBlue==0&&nGreen==0&&nRed==255)
             {
                 rt.data = 3;
                 ROS_INFO("Sector D");
             }
             pub.publish(rt);
         }
        }
        private:
            ros::NodeHandle nh;
            ros::Publisher pub;
            ros::Subscriber sub;
            double pixel_x, pixel_y;
            bool x_inRange{(pixel_x<=14500)}, y_inRange{(pixel_y<=14500)};
            int nBlue, nGreen, nRed;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "sector_publisher");
    sector_publisher sector_publisher;
    ros::spin();
}
