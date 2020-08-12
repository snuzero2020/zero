#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "ros/package.h"

#define USHORT_FACTOR 255   // should be <= 255

//boseol code
int centerline_cost;
int line_cost;
int dilate_pixel_radius;
int spread_pixel_radius;
int color_threshold;

//This function determines how cost will spread
//return * USHORT_FACTOR should be <= 255
inline double spread_ratio(double r){
    return (spread_pixel_radius-r)/(spread_pixel_radius-dilate_pixel_radius);
    //return 10/(r+10-dilate_pixel_radius);
}

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_cost_spreader");
    ros::param::get("/centerline_cost", centerline_cost);
    ros::param::get("/line_cost", line_cost);
    ros::param::get("/dilate_pixel_radius", dilate_pixel_radius);
    ros::param::get("/spread_pixel_radius", spread_pixel_radius);
    ros::param::get("/color_threshold", color_threshold);
    stringstream path_stream;
    path_stream << ros::package::getPath("slam") << "/src/config/FMTC/FMTC_map.png";
    Mat img_input = imread(path_stream.str(),IMREAD_COLOR);
    ROS_INFO("Image loaded");
    uchar* data_input = img_input.data;
    int rows = img_input.rows;
    int cols = img_input.cols;
    Mat img_gray = Mat::zeros(rows, cols, CV_16UC1);
    ushort* data_gray = (ushort*)img_gray.data;
    for(int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            if(data_input[3*(i*cols+j)+1] < color_threshold){    //not blue : black(outer line), red(center line)
                data_gray[i*cols+j] = centerline_cost;
            }else if(data_input[3*(i*cols+j)+2] < color_threshold){  //not red : green(ordinary line)
                data_gray[i*cols+j] = line_cost;
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
    }

    ROS_INFO("Image spreaded");

    Mat img_output = Mat::zeros(rows, cols, CV_8UC1);
    img_spreaded.convertTo(img_output, CV_8UC1, 1.0/USHORT_FACTOR);

    path_stream.str(std::string());
    path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_costmap.png";
    cv::imwrite(path_stream.str(),img_output);
    ROS_INFO("Image saved");
}
