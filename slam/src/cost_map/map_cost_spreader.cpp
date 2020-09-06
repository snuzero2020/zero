#include <chrono>
#include <iostream>

#include "ros/package.h"
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#define USHORT_FACTOR 255   // should be <= 255


//boseol code
int centerline_cost;
int line_cost;
int parkingline_cost;
int dilate_pixel_radius;
int spread_pixel_radius;
int color_threshold;
bool is_kcity;

//This function determines how cost will spread
//return * USHORT_FACTOR should be <= 255
inline double spread_ratio(double r){
    //return (spread_pixel_radius-r)*(spread_pixel_radius-r) / ( (spread_pixel_radius-dilate_pixel_radius)*(spread_pixel_radius-dilate_pixel_radius) );
    return (spread_pixel_radius-r)/(spread_pixel_radius-dilate_pixel_radius);
    //return 10/(r+10-dilate_pixel_radius);
}

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_cost_spreader");
    ros::param::get("/is_kcity", is_kcity);
	ros::param::get("/centerline_cost", centerline_cost);
    ros::param::get("/line_cost", line_cost);
    ros::param::get("/parkingline_cost", parkingline_cost);
    ros::param::get("/dilate_pixel_radius", dilate_pixel_radius);
    ros::param::get("/spread_pixel_radius", spread_pixel_radius);
    ros::param::get("/color_threshold", color_threshold);
    stringstream path_stream;
	if(!is_kcity) path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_map.png";
    else path_stream << ros::package::getPath("slam") << "/config/KCity/KCity.png";
    Mat img_input = imread(path_stream.str(),IMREAD_COLOR);
    if(img_input.empty()){
        ROS_WARN("No image");
    }else{
        if (!is_kcity) ROS_INFO("FMTC image loaded");
        else ROS_INFO("KCity image loaded");
    }
    Mat img_hsv;
    cvtColor(img_input, img_hsv, CV_BGR2HSV);
    uchar* data_hsv = img_hsv.data;
    int rows = img_hsv.rows;
    int cols = img_hsv.cols;
    Mat img_gray = Mat::zeros(rows, cols, CV_16UC1);
    ushort* data_gray = (ushort*)img_gray.data;
    for(int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            int nHue = data_hsv[3*(i*cols+j)];
            int nSaturation = data_hsv[3*(i*cols+j)+1];
            int nValue = data_hsv[3*(i*cols+j)+2];
            if(nSaturation<color_threshold){
                if(nValue<128){
                    data_gray[i*cols+j] = centerline_cost;  //black : borderline    
                }
            }else if(nSaturation>128 && nValue>128){
                if(nHue<color_threshold/2 || nHue>180-color_threshold/2){
                    data_gray[i*cols+j] = centerline_cost;  //red : centerline
                }else if(nHue>65-color_threshold/2 && nHue<65+color_threshold/2){
                    data_gray[i*cols+j] = line_cost;  //green : line
                }else if(nHue>120-color_threshold/2 && nHue<120+color_threshold/2){
                    data_gray[i*cols+j] = parkingline_cost;  //blue : parkingline
                }
            }
        } 
    }
    ROS_INFO("Image preprocessed");

    Mat kernel = Mat::zeros(2*spread_pixel_radius+1, 2*spread_pixel_radius+1, CV_16UC1);
    ushort* data_kernel = (ushort*)kernel.data;
    int kernel_rows = kernel.rows;
    int kernel_cols = kernel.cols;
    for(int i=0; i<kernel_rows; i++){
        for(int j=0; j<kernel_cols; j++){
            double r = sqrt( (i-spread_pixel_radius)*(i-spread_pixel_radius) + (j-spread_pixel_radius)*(j-spread_pixel_radius) );
            if(r<dilate_pixel_radius){
                data_kernel[i*kernel_cols+j] = USHORT_FACTOR;
            }else if(r<spread_pixel_radius){
                data_kernel[i*kernel_cols+j] = USHORT_FACTOR*spread_ratio(r);
            }
        }
    }
    ROS_INFO("Kernel prepared");
    // dilate
    // Mat img_spreaded = Mat::zeros(rows, cols, CV_16UC1);
    // ushort* data_spreaded = (ushort*)img_spreaded.data;
    // for(int ii=spread_pixel_radius; ii<rows-spread_pixel_radius; ii++){
    //     for(int jj=spread_pixel_radius; jj<cols-spread_pixel_radius; jj++){
    //         ushort maxcost = 0;
    //         for(int i=0; i<kernel.rows; i++){
    //             for(int j=0; j<kernel.cols; j++){
    //                 ushort cost = data_gray[(ii+i-spread_pixel_radius)*cols+jj+j-spread_pixel_radius] * data_kernel[i*kernel_cols+j];
    //                 if(cost>maxcost){
    //                     maxcost = cost;
    //                 }
    //            }
    //         }
    //         data_spreaded[ii*cols+jj] = maxcost;
    //     }
    // }
    Mat img_spreaded = Mat::zeros(rows, cols, CV_16UC1);
    ushort* data_spreaded = (ushort*)img_spreaded.data;

    std::chrono::system_clock::time_point time_start = std::chrono::system_clock::now();

    for(int ii=spread_pixel_radius; ii<rows-spread_pixel_radius; ii++){
        for(int jj=spread_pixel_radius; jj<cols-spread_pixel_radius; jj++){
            ushort gray = data_gray[ii*cols+jj];
            if(gray ==0){
                continue;
            }
            for(int i=0; i<kernel.rows; i++){
                for(int j=0; j<kernel.cols; j++){
                    ushort cost = gray*data_kernel[i*kernel_cols+j];
                    if(cost ==0){
                        continue;
                    }
                    ushort* ptr = data_spreaded + (ii+i-spread_pixel_radius)*cols + jj+j-spread_pixel_radius;
                    if(*ptr < cost){
                        *ptr = cost;
                    }
               }
            }
        }

        // calculate rest time
        std::chrono::duration<double> elipsed_time = std::chrono::system_clock::now() - time_start;
        std::chrono::milliseconds elipsed_time_mill  = std::chrono::duration_cast<std::chrono::milliseconds>(elipsed_time);
        double rest_time_mill = elipsed_time_mill.count() * (rows-spread_pixel_radius-ii) / (ii-spread_pixel_radius + 1);

        int rest_time_hour = static_cast<int>(rest_time_mill) / 3600000;
        int rest_time_minute = (static_cast<int>(rest_time_mill) - (rest_time_hour * 3600000)) / 60000;
        int rest_time_second = ((static_cast<int>(rest_time_mill) - (rest_time_hour * 3600000)) - rest_time_minute * 60000) / 1000;

        std::string rest_time_minute_string = to_string(rest_time_minute);
        std::string rest_time_second_string = to_string(rest_time_second);

        if (rest_time_minute < 10) rest_time_minute_string = "0" + to_string(rest_time_minute);

        if (rest_time_second < 10) rest_time_second_string = "0" + to_string(rest_time_second);

        int percent = (ii-spread_pixel_radius) * 100 / (rows-2*spread_pixel_radius);

        //std::cout << elipsed_time_mill.count() << '\n';
        //std::cout << rest_time_mill << "\n";
        std::cout << "Complete after " << rest_time_hour << ":" << rest_time_minute_string << ":" << rest_time_second_string << "(" << percent << "% completed)    \r";
    }

    ROS_INFO("Image spreaded");

    Mat img_output = Mat::zeros(rows, cols, CV_8UC1);
    img_spreaded.convertTo(img_output, CV_8UC1, 1.0/USHORT_FACTOR);

    path_stream.str(std::string());
    if(!is_kcity) path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_costmap.png";
    else path_stream << ros::package::getPath("slam") << "/config/KCity/KCity_costmap.png";
    cv::imwrite(path_stream.str(),img_output);
    ROS_INFO("Image saved");
}
