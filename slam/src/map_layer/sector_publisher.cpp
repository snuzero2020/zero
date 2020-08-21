#include <iostream>
#include <string>
#include <vector>

#include "std_msgs/Int32.h"

#include "slam/Data.h"
#include "slam/Pixel.h"

#include "ros/package.h"
#include "ros/ros.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "XYToPixel.h"


class sector_publisher{
    private:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        int pixel_x, pixel_y;
        int nBlue, nGreen, nRed;
        bool is_kcity;


    public:
        std::stringstream path_stream;
        cv::Mat color_map;

        //cv::Mat color_map = cv::imread("/home/healthykim/catkin_ws/src/zero/slam/config/KCity/KCity_color_map.png", cv::IMREAD_COLOR);

        sector_publisher() {
            ros::param::get("/is_kcity", is_kcity);
            if(is_kcity==true){
                path_stream << ros::package::getPath("slam") << "/config/KCity/KCity_color_map.png";
                color_map = cv::imread(path_stream.str());
                if(!color_map.empty()){
                    ROS_INFO("KCity loaded");
                }  
            }
            else if(is_kcity==false){
                path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_color_map.png";
                color_map = cv::imread(path_stream.str()); 
                if(!color_map.empty()){
                    ROS_INFO("FMTC loaded");
                }     
            }  
            pub = nh.advertise<std_msgs::Int32>("/sector_info_test", 1);
            sub = nh.subscribe("/filtered_data",2, &sector_publisher::callback, this);
        }

        //void callback(const slam::Pixel Data){
        void callback(const slam::Data::ConstPtr& msg){
            bool x_inRange, y_inRange;
            XYToPixel(pixel_y, pixel_x, msg->x, msg->y, is_kcity); // pixel_y here is x in cv graphics and column in cv Mat
            
            if(is_kcity==true){
            x_inRange ={pixel_x<=22489 && pixel_x > 0};
            y_inRange ={pixel_y<=8273 && pixel_y > 0};
            }

            if(is_kcity==false){
            x_inRange ={pixel_x<=14226 && pixel_x > 0};
            y_inRange ={pixel_y<=12072 && pixel_y > 0};
            }

            std::cout<<"Pixel information is loaded: "<<pixel_x<<", "<<pixel_y <<std::endl;

            if(x_inRange&&y_inRange){
                std::cout<<"on map"<<std::endl;
                //get the RGB information of a pixel 
                nBlue = color_map.at<cv::Vec3b>(pixel_x, pixel_y)[0];
                nGreen = color_map.at<cv::Vec3b>(pixel_x, pixel_y)[1];
                nRed = color_map.at<cv::Vec3b>(pixel_x, pixel_y)[2];
                
                std::cout<<nBlue<<" "<<nGreen<<" "<<nRed<<std::endl;

                //publish
                // in sector A ==> pub 0
                //           B ==> pub 1
                //           C ==> pub 2
                //           D ==> pub 3

                std_msgs::Int32 rt;

                if(nBlue==0&&nGreen==0&&nRed==0)
                {
                    rt.data = 0;
                    ROS_INFO("Sector A");
                }
                else if(nBlue==255&&nGreen==255&&nRed==255){
                    rt.data = 0;
                    ROS_INFO("Sector A with curve");
                }

                else if(nGreen==255){
                    if(nBlue == 0 && nRed ==0){
                        rt.data = 2;
                        ROS_INFO("Sector C");
                    }
                    else if(nRed == 0 && nBlue == 140){
                        rt.data = 5;
                        ROS_INFO("Sector F");
                    }
                    else if(nRed == 100 && nBlue == 140){
                        rt.data = 5|(1<<4);
                        ROS_INFO("Sector F'");
                    }
                    else if(nRed == 140 && nBlue ==0){
                        rt.data = 7;
                        ROS_INFO("Sector H");
                    }
                    else if(nRed == 140 && nBlue == 100){
                        rt.data = 7|(1<<4);
                        ROS_INFO("Sector H'");
                    }
                    else if(nRed == 0 && nBlue == 255){
                        rt.data = 11;
                        ROS_INFO("Sector L");
                    }
                    else if(nRed == 100 && nBlue == 255){
                        rt.data = 11|(1<<4);
                        ROS_INFO("Sector L'");
                    }
                    else{
                        rt.data = -1;
                        ROS_INFO("Sector X");
                    }
                }
                
                else if(nBlue==255)
                {
                    if(nRed==0 && nGreen==0){
                        rt.data =  3;
                        ROS_INFO("Sector D");
                    }
                    else if(nGreen==100 && nRed==100){
                        rt.data = (3)|(1<<4);
                        ROS_INFO("Sector D'");
                    }
                    else if(nGreen == 140 && nRed == 0){
                        rt.data = 8;
                        ROS_INFO("Sector I");
                    }
                    else if(nGreen == 140 && nRed == 100){
                        rt.data = 8|(1<<4);
                        ROS_INFO("Sector I'");
                    }
                    else if(nRed == 140 && nGreen == 0){
                        rt.data = 9;
                        ROS_INFO("Sector J");
                        
                    }
                    else if(nRed == 140 && nGreen == 100){
                        rt.data = 9|(1<<4);
                        ROS_INFO("Sector J'");
                    }
                    else{
                        rt.data = -1;
                        ROS_INFO("Sector X");                    
                    }
                }

                else if(nRed==255)
                {
                    if(nBlue==0 && nGreen==0)
                    {
                        rt.data = 1;
                        ROS_INFO("Sector B");
                     }
                    else if(nBlue==0 && nGreen==140){
                        rt.data = 4;
                        ROS_INFO("Sector E");
                    }
                    else if(nBlue==100 && nGreen==140){
                        rt.data = 4|(1<<4);
                        ROS_INFO("Sector E'");
                    }

                    else if(nBlue==140 && nGreen==0){
                        rt.data = 6;
                        ROS_INFO("Sector G");
                    }
                    else if(nBlue==140 && nGreen == 100){
                        rt.data = 6|(1<<4);
                        ROS_INFO("Sector G'");
                    }
                    else if(nGreen==255 && nBlue == 0){
                        rt.data = 10;
                        ROS_INFO("Sector K");
                    }
                    else if(nGreen==255 && nBlue == 100){
                        rt.data = 10|(1<<4);
                        ROS_INFO("Sector K'");
                    }
                    else{
                        rt.data = -1;
                        ROS_INFO("Sector X");                    
                    }
                } 
                else{
                    rt.data = -1;
                    ROS_INFO("Sector X");                    
                }
                pub.publish(rt);
                 //previous_data = rt.data;
                 
            }
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "sector_publisher");
    sector_publisher sector_publisher;
    ros::spin();
}
