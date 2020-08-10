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
    Mat velocity_map = Mat(15000,15000,CV_8UC3, Scalar(0,0,0));
    stringstream path_stream1;
    stringstream path_stream2;
    path_stream1 << ros::package::getPath("slam") << "/config/color_map.png";
    path_stream2 << ros::package::getPath("slam") << "/config/discontinous_velocity_map.png";
    color_map = imread(path_stream1.str());
    ROS_INFO("color map loaded");

    //convert
    for(int i=0; i<15000; i++){
        for(int j=0; j<15000; j++){
             std::cout<<"on map"<<std::endl;
             int nBlue, nGreen, nRed;
             nBlue = color_map.at<cv::Vec3b>(i, j)[0];
             nGreen = color_map.at<cv::Vec3b>(i, j)[1];
             nRed = color_map.at<cv::Vec3b>(i, j)[2];

             if(nBlue==0&&nGreen==0&&nRed==0)
             {
                velocity_map.at<cv::Vec3b>(i, j)[0] = 3;
             }
             if(nBlue==0&&nGreen==255&&nRed==0)
             {
                velocity_map.at<cv::Vec3b>(i, j)[0] = 2;
             }
             if(nBlue==255&&nGreen==0&&nRed==0)
             {
                velocity_map.at<cv::Vec3b>(i, j)[0] = 2;
             }
             if(nBlue==0&&nGreen==0&&nRed==255)
             {
                velocity_map.at<cv::Vec3b>(i, j)[0] = 2;
             }
        }
    }
    imwrite(path_stream2.str(), velocity_map);
    ROS_INFO("discontinous velocity map is saved");
    Gaussianblur(velocity_map, velocity_map, Size(60,60), 1.5, 0);
    path_stream2 << ros::package::getPath("slam") << "/config/velocity_map.png";
    imwrite(path_stream2.str(), velocity_map);
}