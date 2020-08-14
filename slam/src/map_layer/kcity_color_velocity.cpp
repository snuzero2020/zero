#include <ros/ros.h>
#include "ros/package.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <vector>

using namespace std;
using namespace cv;

int main(int argc, char**argv){
    Mat color_map;
    Mat discrete_velocity_map(15000,15000, CV_8UC3);
    Mat velocity_map(15000,15000, CV_8UC3);
    stringstream path_stream1;
    stringstream path_stream2;
    stringstream path_stream3;
    path_stream1 << ros::package::getPath("slam") << "/config/KCity/KCity_color_map.png";
    color_map = imread(path_stream1.str());
    ROS_INFO("color map loaded");

    //convert
    for(int i=0; i<15000; i++){
        for(int j=0; j<15000; j++){
             int nBlue, nGreen, nRed;
             nBlue = color_map.at<Vec3b>(i, j)[0];
             nGreen = color_map.at<Vec3b>(i, j)[1];
             nRed = color_map.at<Vec3b>(i, j)[2];

             if(nBlue==0&&nGreen==0&&nRed==0)
             {
                discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 255;
             }

             else{
                discrete_velocity_map.at<cv::Vec3b>(i,j)[0] = 170;
             }

             /*
             if(nGreen>=200)
             {
                if(nBlue==55)
                {
                   discrete_velocity_map.at<cv::Vec3b>(i,j)[0] = 170;
                }
                else
                {
                   discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 170;
                }
             }

             if(nBlue==255)
             {
                discrete_velocity_map.at<cv::Vec3b>(i,j)[0] = 170;
             }
             if(nRed==255)
             {
                discrete_velocity_map.at<cv::Vec3b>(i, j)[0] = 170;
             }
             */
        }
    }

    path_stream2 << ros::package::getPath("slam")<<"/config/KCity/KCity_discrete_velocity_map.png";
    cv::imwrite(path_stream2.str(), discrete_velocity_map);
    ROS_INFO("discrete velocity map is saved");
    GaussianBlur(discrete_velocity_map, velocity_map, Size(33,33), 20, 0);
    path_stream3 << ros::package::getPath("slam")<<"/config/KCity/KCity_velocity_map.png";
    cv::imwrite(path_stream3.str(), velocity_map);
    ROS_INFO("velocity map is saved");

}