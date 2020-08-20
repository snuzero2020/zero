#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "ros/package.h"

int color_threshold;

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_lane_extractor");
    ros::param::get("/color_threshold", color_threshold);
    stringstream path_stream;
    path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_map_for_costmap.png";
    Mat img_input = imread(path_stream.str(),IMREAD_COLOR);
    if(img_input.empty()){
        ROS_WARN("No image");
    }else{
        ROS_INFO("Image loaded");
    }
    Mat img_hsv;
    cvtColor(img_input, img_hsv, CV_BGR2HSV);
    uchar* data_hsv = img_hsv.data;
    int rows = img_hsv.rows;
    int cols = img_hsv.cols;
    Mat img_gray = Mat::zeros(rows, cols, CV_8UC1);
    uchar* data_gray = img_gray.data;
    for(int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            int nHue = data_hsv[3*(i*cols+j)];
            int nSaturation = data_hsv[3*(i*cols+j)+1];
            int nValue = data_hsv[3*(i*cols+j)+2];
            if(nSaturation<color_threshold){
                if(nValue<128){
                    data_gray[i*cols+j] = 255;  //black : borderline    
                }
            }else if(nSaturation>128 && nValue>128){
                if(nHue<color_threshold/2 || nHue>180-color_threshold/2){
                    data_gray[i*cols+j] = 255;  //red : centerline
                }else if(nHue>65-color_threshold/2 && nHue<65+color_threshold/2){
                    data_gray[i*cols+j] = 255;  //green : line
                }
            }
        } 
    }
    path_stream.str(std::string());
    path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_lane.png";
    cv::imwrite(path_stream.str(),img_gray);
    ROS_INFO("Image saved");
}
