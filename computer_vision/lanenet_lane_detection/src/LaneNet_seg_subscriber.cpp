#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

void seg_print_callback(const sensor_msgs::ImageConstPtr &msg){
    cv_bridge::CvImagePtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat seg_img = cv_ptr->image;
    cv::imshow("seg_img", seg_img);
    cv::waitKey(1);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "LaneNet_subscriber_node");
    ros::NodeHandle nh;
    ros::Subscriber seg_sub = nh.subscribe("/lane_seg_topic", 1, &seg_print_callback);
    ros::spin();
    return 0;
}