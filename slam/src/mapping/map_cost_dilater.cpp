#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include "ros/package.h"

#define CENTERLINE 100  //cost on centerline
#define LINE 40         //cost on ordinary line
#define PUREDILATE 7    //pixel size to pre-dilate
#define MAXPIX 67       //pixel size to spread cost
#define COLORTHRESHOLD 180
#define UNIT 255        //ushort value

//This function determines how cost will spread
inline double dilate_ratio(double r){
    return (MAXPIX-r)/(MAXPIX-PUREDILATE);
    //return 10/(r+10-PUREDILATE);
}

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_cost_dilater");
    stringstream path_stream;
    path_stream << ros::package::getPath("slam") << "/src/mapping/map.png";
    Mat img_input = imread(path_stream.str(),IMREAD_UNCHANGED);
    ROS_INFO("Image loaded");

    uchar* data_input = img_input.data;
    int rows = img_input.rows;
    int cols = img_input.cols;
    Mat img_gray = Mat::zeros(rows, cols, CV_16UC1);
    ushort* data_gray = (ushort*)img_gray.data;
    for(int i=0; i<rows; i++){
        for(int j=0; j<cols; j++){
            if(data_input[4*(i*cols+j)+3] > COLORTHRESHOLD){    //not transparent
                if(data_input[4*(i*cols+j)+1] < COLORTHRESHOLD){    //not blue : black(outer line), red(center line)
                    data_gray[i*cols+j] = CENTERLINE;
                }else if(data_input[4*(i*cols+j)+2] < COLORTHRESHOLD){  //not red : green(ordinary line)
                    data_gray[i*cols+j] = LINE;
                }
            }
        } 
    }
    ROS_INFO("Image preprocessed");

    Mat kernel = Mat::zeros(2*MAXPIX+1, 2*MAXPIX+1, CV_16UC1);
    ushort* data_kernel = (ushort*)kernel.data;
    int kernel_rows = kernel.rows;
    int kernel_cols = kernel.cols;
    for(int i=0; i<kernel_rows; i++){
        for(int j=0; j<kernel_cols; j++){
            double r = sqrt( (i-MAXPIX)*(i-MAXPIX) + (j-MAXPIX)*(j-MAXPIX) );
            if(r<PUREDILATE){
                data_kernel[i*kernel_cols+j] = UNIT;
            }else if(r<MAXPIX){
                data_kernel[i*kernel_cols+j] = UNIT*dilate_ratio(r);
            }
        }
    }
    ROS_INFO("Kernel prepared");
    // dilate
    // Mat img_dilated = Mat::zeros(rows, cols, CV_16UC1);
    // ushort* data_dilated = (ushort*)img_dilated.data;
    // for(int ii=MAXPIX; ii<rows-MAXPIX; ii++){
    //     for(int jj=MAXPIX; jj<cols-MAXPIX; jj++){
    //         ushort maxcost = 0;
    //         for(int i=0; i<kernel.rows; i++){
    //             for(int j=0; j<kernel.cols; j++){
    //                 ushort cost = data_gray[(ii+i-MAXPIX)*cols+jj+j-MAXPIX] * data_kernel[i*kernel_cols+j];
    //                 if(cost>maxcost){
    //                     maxcost = cost;
    //                 }
    //            }
    //         }
    //         data_dilated[ii*cols+jj] = maxcost;
    //     }
    // }
    Mat img_dilated = Mat::zeros(rows, cols, CV_16UC1);
    ushort* data_dilated = (ushort*)img_dilated.data;
    for(int ii=MAXPIX; ii<rows-MAXPIX; ii++){
        for(int jj=MAXPIX; jj<cols-MAXPIX; jj++){
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
                    ushort* ptr = data_dilated + (ii+i-MAXPIX)*cols + jj+j-MAXPIX;
                    if(*ptr < cost){
                        *ptr = cost;
                    }
               }
            }
        }
    }

    ROS_INFO("Image dilated");

    Mat img_output = Mat::zeros(rows, cols, CV_8UC1);
    img_dilated.convertTo(img_output, CV_8UC1, 1.0/UNIT);

    path_stream.str(std::string());
    path_stream << ros::package::getPath("slam") << "/src/mapping/costmap.png";
    cv::imwrite(path_stream.str(),img_output);
    ROS_INFO("Image saved");
}