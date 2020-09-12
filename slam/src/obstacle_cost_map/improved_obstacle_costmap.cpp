#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <math.h>

#include "geometry_msgs/Point.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/OccupancyGrid.h"

#include "slam/Cluster.h"
#include "slam/Lidar.h"
#include "slam/Data.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "Eigen/Eigen"
#include "Eigen/Core"
#include "Eigen/Dense"

#include "XYToPixel.h"
#include "opencv2/opencv.hpp"
#include "opencv2/core/eigen.hpp"
#include "cv_bridge/cv_bridge.h"

using namespace std;
using namespace cv;
using namespace Eigen;

class DecayingCostmap{
    public:
    DecayingCostmap(){
        pub_ = nh_.advertise<sensor_msgs::Image>("/obstacle_map/decaying_costmap",10);
        pub_occupancy_ = nh_.advertise<nav_msgs::OccupancyGrid>("/obstacle_occupancy/decaying_costmap",10);
        sub_img_ = nh_.subscribe("/obstacle_map/costmap", 1, &DecayingCostmap::img_callback, this);
        sub_pose_ = nh_.subscribe("/filtered_data", 1, &DecayingCostmap::pose_callback, this);
        rt_costmap = Mat::zeros(432, 432, CV_8U);
        warp_costmap = Mat::zeros(432, 432, CV_8U);
        pad_size = 66;
        namedWindow("decaying_costmap");
    }

    //USE VECTOR'S ITERATION    
    void pose_callback(const slam::Data::ConstPtr& msg){ 
        pose.x = msg->x;
        pose.y = msg->y;
        pose.theta = msg->theta;
    }

    void img_callback(const sensor_msgs::Image::ConstPtr &msg){
        clock_t begin = clock();
        cv_bridge::CvImagePtr map_ptr;
        try{
            map_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        costmap = map_ptr -> image;

        // Make current_pose, past_pose
        make_poses_list(pose);
        if (poses_list.size() == 2)
        {
            //warpAffine(rt_costmap, warp_costmap, calibrate_map(poses_list[1], poses_list[0]), costmap.size());
            warpAffine(rt_costmap, rt_costmap, calibrate_map(poses_list[1], poses_list[0]), costmap.size());
            // Interpolate the influence of the consequential costmaps
            for(int i = 0 ; i < costmap.rows ; i++){
                alpha = 1;
                beta = 0.8;
                for(int j = 0; j < costmap.cols; j++){
                    if(j > costmap.cols - 100) {
                        alpha = 0.65;
                        beta = 0.97;
                    }
                    int final_cost = alpha * costmap.at<uchar>(j,i) + beta * rt_costmap.at<uchar>(j,i);

                    if(final_cost <= 100){
                        rt_costmap.at<uchar>(j,i) = final_cost;
                    }
                    else rt_costmap.at<uchar>(j,i) = 100;
                    // rt_costmap.at<uchar>(j,i) = saturate_cast<uchar>(alpha * costmap.at<uchar>(j,i) + beta * rt_costmap.at<uchar>(j,i));
                }
            }
        }

        costmap_sliced = rt_costmap(Range(pad_size, rt_costmap.rows - pad_size), Range(pad_size, rt_costmap.rows - pad_size));
        imshow("sliced_costmap", costmap_sliced);
        // imshow("decaying_costmap", rt_costmap);
        int key = waitKey(30);

        //IMAGE_PUBLISHING
        sensor_msgs::Image rt;
        std_msgs::Header header;
        header.seq = msg->header.seq;
        header.stamp = ros::Time::now();
        pub_img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, costmap_sliced);
        pub_img_bridge.toImageMsg(rt);
        pub_.publish(rt);
        
        //Occupacncy Grid Publishing
        nav_msgs::OccupancyGrid costmap2pp;
		costmap2pp.info.width = 300;
		costmap2pp.info.height = 300;

		for (int i = 1; i < 301; i++){
			for (int j = 1; j < 301; j++) costmap2pp.data.push_back(static_cast<int8_t>(costmap.at<uchar>(300-i,300-j)));
		}
		pub_occupancy_.publish(costmap2pp);
        
        //About time
        clock_t end = clock();

        if(max_time < double(end-begin)/CLOCKS_PER_SEC){
            max_time = double(end-begin)/CLOCKS_PER_SEC;
        }
        //ROS_INFO("max elapsed time (2): %lf", max_time);
        //ROS_INFO("elapsed time (2): %lf", double(end-begin)/CLOCKS_PER_SEC);
        mainclock = ros::Time::now();
    }

    // Using current pose and past pose, calibrate past lidar data into current pose's view.
    // 1. Reference Frame Change between past and current pose
    // 2. Reference Frame Change between car's point(300,150) and image pixel zero point(0,0)
    // 3. Make Jordan Similar Matrix and compute it
    // 4. Return Calibration Matrix 

    Mat calibrate_map(slam::Data current_pose, slam::Data past_pose){
        int current_pixel_x, current_pixel_y, past_pixel_x, past_pixel_y, trans_x, trans_y;
        double current_theta, past_theta;
        XYToPixel(current_pixel_x, current_pixel_y, current_pose.x, current_pose.y, false);
        XYToPixel(past_pixel_x, past_pixel_y, past_pose.x, past_pose.y, false);
        current_theta = current_pose.theta / 180 * M_PI;
        past_theta = past_pose.theta / 180 * M_PI;
        
        //1. Reference frame change between past and current pose
        MatrixXd calibrationMatrix(3,3);
        MatrixXd T_current(3,3);
        MatrixXd T_past(3,3);
        MatrixXd finalMatrix_Eigen(3,3);
        Mat_<float> finalMatrix_CV;
        Mat_<float> finalMatrix_CV_test;

        //T_ab = T_ao (T_current inverse) * T_ob (T_past)
        T_current << cos(current_theta), -sin(current_theta), current_pixel_x,
                    sin(current_theta), cos(current_theta), current_pixel_y,
                    0, 0, 1;

        T_past << cos(past_theta), -sin(past_theta), past_pixel_x,
                    sin(past_theta), cos(past_theta), past_pixel_y,
                    0, 0, 1; 
        
        calibrationMatrix = T_past.inverse() * T_current;

        //2. Reference frame change between car's point and zero point in the image pixel world
        MatrixXd moveZeropoint(3,3);
        moveZeropoint << 1, 0, -216,
                         0, 1, -366,
                         0, 0, 1;
        //3. Make Jordan Similar Matrix P-1AP
        finalMatrix_Eigen = moveZeropoint.inverse() * calibrationMatrix * moveZeropoint;
        //4. Put Eigen to CV Function
        eigen2cv(finalMatrix_Eigen, finalMatrix_CV);
        //5. Pop up last row
        finalMatrix_CV_test = finalMatrix_CV(Range(0,2),Range(0,3));
        return finalMatrix_CV_test;
    }

    // Put car's pose into the pose vector (size : 2)
    // 1. Delete old one 2. Push back new pose
    // Vector of past pose and current pose
    void make_poses_list(slam::Data pose){
        if(poses_list.size() < 2) poses_list.push_back(pose);
        else{
            vector<slam::Data>::iterator it = poses_list.begin();
            it = poses_list.erase(it);
            poses_list.push_back(pose);
        }
    }

    // If no topics are subscribed for 2 seconds, then quit!
    void quit_if_end(){
        if((ros::Time::now() - mainclock).sec > 2){
            destroyWindow("sliced_costmap");
            destroyWindow("decaying_costmap");
        }
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_occupancy_;
    ros::Subscriber sub_img_;
    ros::Subscriber sub_pose_;
    ros::Time mainclock;
    double alpha, beta;

    int pad_size;
    slam::Data pose;
    Mat rt_costmap;
    Mat costmap;
    Mat warp_costmap;
    Mat costmap_sliced;
    // vector<Mat> costmaps_list;
    vector<slam::Data> poses_list;

    cv_bridge::CvImage pub_img_bridge;

    double max_time;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "decaying_costmap_publisher");
    DecayingCostmap decaying_costmap;
    while(ros::ok()){
        decaying_costmap.quit_if_end();
        ros::spinOnce();
    }
    return 0;
}
