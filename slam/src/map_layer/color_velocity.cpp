#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "ros/package.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;


int main(int argc, char**argv){
    Mat color_map;
    Mat discrete_velocity_map(15000, 15000, CV_8UC3);
    Mat velocity_map(15000, 15000, CV_8UC3);
    stringstream path_stream1;
    stringstream path_stream2;
    stringstream path_stream3;
    bool is_kcity;

    ros::param::get("/is_kcity", is_kcity);
    is_kcity = true;
    if(is_kcity==true){
       path_stream1 << ros::package::getPath("slam") << "/config/KCity/KCity_color_map_v6.png";
       path_stream2 << ros::package::getPath("slam")<<"/config/KCity/KCity_discrete_velocity_map.png";
       path_stream3 << ros::package::getPath("slam")<<"/config/KCity/KCity_velocity_map_6.png";
       color_map = imread(path_stream1.str());
       if(!color_map.empty()){
          ROS_INFO("kcity color map loaded");
       }

    }
    else if(is_kcity==false){
       path_stream1 << ros::package::getPath("slam") << "/config/FMTC/FMTC_color_map_v.png";
       path_stream2 << ros::package::getPath("slam")<<"/config/FMTC/FMTC_discrete_velocity_map.png";
       path_stream3 << ros::package::getPath("slam")<<"/config/FMTC/new/FMTC_velocity_map_555_5.png";
       color_map = imread(path_stream1.str());
       if(!color_map.empty()){
          ROS_INFO("FMTC color map loaded");
       }
    }

    resize(discrete_velocity_map, discrete_velocity_map, Size(color_map.cols, color_map.rows), 0, 0, CV_INTER_NN);
    resize(velocity_map, velocity_map, Size(color_map.cols, color_map.rows),0,0,CV_INTER_NN);

    /*
    //5
    for(int j=0; j<color_map.cols; j++){
        for(int i=0; i<color_map.rows; i++){
             int nBlue, nGreen, nRed;
             nBlue = color_map.at<Vec3b>(i, j)[0];
             nGreen = color_map.at<Vec3b>(i, j)[1];
             nRed = color_map.at<Vec3b>(i, j)[2];
            
            if(nBlue==0&&nGreen==0&&nRed==0){
                discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 85;
                //Driving section
            }
            else if(nBlue==255&&nGreen==255&&nRed==255){
                discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 85;
            }
            else if(nGreen==255){
                if(nBlue == 0 && nRed ==0){
                 discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 127;
                  //Children
                }
                if(nRed == 255 && nBlue == 0){
                 discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 127;
                 //intersection buffer 2
                }
                if(nRed == 140 && nBlue == 140){
                 discrete_velocity_map.at<cv::Vec3b>(i,j)[0] = 212;
                 //buffer 1(After Children)
                }
                if(nRed == 0 && nBlue == 255){
                 discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 255;
                 //buffer 2(After Children)
                }
            }

            else if(nBlue==255){
                if(nRed==140 && nGreen == 140){
                 discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 106;                 
                    //intersection buffer 1
                }
                if(nRed==0 && nGreen==0){
                 discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 212;
                 //intersection buffer 3
                }
                if(nGreen==100 && nRed==100){
                 discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 127;
                 //intersection
                }
            } 

            else
             discrete_velocity_map.at<cv::Vec3b>(i,j)[0]= 85;             
        }
    }  
    */  


    //4 at KCity
    for(int j=0; j<color_map.cols; j++){
        for(int i=0; i<color_map.rows; i++){
             int nBlue, nGreen, nRed;
             nBlue = color_map.at<Vec3b>(i, j)[0];
             nGreen = color_map.at<Vec3b>(i, j)[1];
             nRed = color_map.at<Vec3b>(i, j)[2];
            
            if(nBlue==0&&nGreen==0&&nRed==0){
                discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 85;
                //Driving section
            }
            else if(nBlue==255&&nGreen==255&&nRed==255){
                discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 85;
            }

            else if(nGreen==255){
                if(nRed == 255 && nBlue == 0){
                 discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 127;
                 //intersection buffer 2 //127->2.6 //110->3
                }
            }

            else if(nBlue==255){
                
                if(nRed==0 && nGreen==0){
                 discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 110;
                 //intersection buffer 1 //170->2 //110->3 //76 -> 4.5
                }
                
                if(nGreen==100 && nRed==100){
                 discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 85;
                 //intersection
                }
            } 

            else if(nRed==255){
                if(nBlue==0 && nGreen==0){
                 discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 68;
                 //velocity 5
                }      
            }

            else
             discrete_velocity_map.at<cv::Vec3b>(i,j)[0]= 76;             
        }
    }   
     




    //cv::Mat kernel=cv::getStructuringElement(cv::MORPH_RECT, cv::Size(99,99));
    //cv::dilate(discrete_velocity_map, discrete_velocity_map, kernel, cv::Point(-1,-1), 1);
    imwrite(path_stream2.str(), discrete_velocity_map);
    ROS_INFO("discrete velocity map is saved");
    //GaussianBlur(discrete_velocity_map, velocity_map, Size(267, 267), 2000, 0);
    Mat linearKernel= Mat::ones(555,555,CV_32F)/(float)(555*555);
    filter2D(discrete_velocity_map, velocity_map, -1, linearKernel, Point(-1,-1));
    imwrite(path_stream3.str(), velocity_map);
    ROS_INFO("velocity map is saved");

}