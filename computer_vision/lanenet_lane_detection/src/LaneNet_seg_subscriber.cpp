#include "ros/ros.h"
#include <iostream>
#include "lanenet_lane_detection/lanenet_msg.h"
#include <opencv2/opencv.hpp>
#include <ctime>

using namespace cv;

void lanenet_callback(const lanenet_lane_detection::lanenet_msg::ConstPtr &msg){
    clock_t start;
    start = clock();
    
    cv::Mat left_binary_seg = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat right_binary_seg = cv::Mat::zeros(480, 640, CV_8UC1);
    
    float decode[2][2][480][640];

    for(int h=0; h<480; h++){
        for(int w=0; w<640; w++){
            if(msg->data[h*640 + w] < msg->data[480*640 + h*640 + w]){
                left_binary_seg.at<uchar>(h,w) = 255;
            }
            if(msg->data[2*640*480 + h*640 + w] < msg->data[2*640*480 + 480*640 + h*640 + w]){
                right_binary_seg.at<uchar>(h,w) = 255;
            }
        }
    }


    std::cout<<"C++ lane_postprocessing time : "<<(double)(clock()-start)/CLOCKS_PER_SEC<<std::endl;
    cv::imshow("left_binary_seg", left_binary_seg);
    cv::imshow("right_binary_seg", right_binary_seg);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lanenet_seg_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber lanenet_sub = nh.subscribe("/lane_seg_topic",100, lanenet_callback);

    ros::spin();

    return 0;
}