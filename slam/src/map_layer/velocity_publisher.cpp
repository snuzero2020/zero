#include <iostream>
#include <string>
#include <vector>

#include "std_msgs/Float64.h"

#include "slam/Pixel.h"
#include "slam/Data.h"

#include "ros/package.h"
#include "ros/ros.h"
#include "opencv2/opencv.hpp"

#include "XYToPixel.h"


class velocity_publisher{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        int pixel_x, pixel_y;
        double recommended_velocity;
        double max_velocity;
        bool is_kcity;
        int sigma;
        int kernelSize;

    public:
        cv::Mat velocity_map;
        std::stringstream path_stream;

        velocity_publisher(){
            ros::param::get("/is_kcity", is_kcity);
            ros::param::get("/sigma", sigma);
            ros::param::get("/kernelSize", kernelSize);

            if(is_kcity==true){
            	path_stream << ros::package::getPath("slam") << "/config/KCity/KCity_velocity_map.png";
                velocity_map = cv::imread(path_stream.str(), cv::IMREAD_COLOR);

                if(!velocity_map.empty()){
                    ROS_INFO("KCity loaded");
                }   
            }
            else if(is_kcity==false){
            	path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_velocity_map_"<<kernelSize<<"_"<<sigma<<".png";
                velocity_map = cv::imread(path_stream.str(), cv::IMREAD_COLOR);  
                   if(!velocity_map.empty()){
                        ROS_INFO("FMTC loaded");
                    }   
            }
 
            pub = nh.advertise<std_msgs::Float64>("/recommended_velocity", 2);
            sub = nh.subscribe("/filtered_data",2, &velocity_publisher::callback, this);
        }

        void callback(const slam::Data::ConstPtr& msg){
            bool x_inRange, y_inRange;
            ros::param::get("/max_velocity", max_velocity);
 
            XYToPixel(pixel_y, pixel_x, msg->x, msg->y, is_kcity); // pixel_y here is x in cv graphics and column in cv Mat
            
            if(is_kcity==true){
            x_inRange ={pixel_x<=22489 && pixel_x > 0};
            y_inRange ={pixel_y<=8273 && pixel_y > 0};
            }

            if(is_kcity==false){
            x_inRange ={pixel_x<=14226 && pixel_x > 0};
            y_inRange ={pixel_y<=12072 && pixel_y > 0};
            }

            std::cout<<"Pixel information is loaded: "<<pixel_x<<", "<<pixel_y<<std::endl;

            if(x_inRange&&y_inRange){
                std::cout<<"on map"<<std::endl;
                recommended_velocity = velocity_map.at<cv::Vec3b>(pixel_x, pixel_y)[0];
                std_msgs::Float64 rt;
                rt.data = recommended_velocity/255*max_velocity;
                pub.publish(rt);
            }
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "velocity_publisher");
    velocity_publisher velocity_publisher;
    ros::spin();
}

