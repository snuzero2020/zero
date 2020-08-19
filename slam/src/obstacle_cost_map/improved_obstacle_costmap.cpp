#include <algorithm>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "geometry_msgs/Point.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/OccupancyGrid.h"

#include "slam/Cluster.h"
#include "slam/Lidar.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "Eigen/Eigen"
#include "XYToPixel.h"


class ImprovedCostmap{
    public:
    ImprovedCostmap(){
        pub_ = nh_.advertise<sensor_msgs::Image>("obstacle_map/costmap",10);
        sub_img_ = nh_.subscribe("/obstacle_map/costmap", 1, &ImprovedCostmap::callback, this);
        sub_pose_ = nh_.subscribe("/filtered_data", 1, &ImprovedCostmap::pose_callback,this);
        final_costmap = Mat::zeros(300, 300, CV_8UC3);
    }

    //USE VECTOR'S ITERATION    
    void pose_callback(const slam::Data::ConstPtr& msg){ 
        pose.x = msg->x;
        pose.y = msg->y;
        pose.theta = msg->theta;
    }

    void img_callback(const sensor_msgs::Image::ConstPtr &msg){
        current_pose = pose;
        cv_bridge::CvImagePtr map_ptr;
        try{
            map_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        Mat costmap = map_ptr -> image;
        make_poses_list(pose);
        calibrate_map(final_costmap, poses_list[0], poses_list[1]);

        for(int i = 0 ; i < costmap.rows ; i++){
            for(int j = 0; j < costmap.cols; j++){
                final_costmap.at<uchar>(j,i) = gamma * costmap.at<uchar>(j,i) + (1-gamma) * final_costmap.at<uchar>(j,i);
            }
        }
        imshow("original_costmap", costmap);
        imshow("improved_costmap",final_costmap);
    }

    Mat calibrate_map(Mat map, slam::Data current_pose, slam::Data past_pose){
        int current_pixel_x, current_pixel_y, past_pixel_x, past_pixel_y, trans_x, trans_y;
        double theta;
        XYToPixel(current_pixel_x, current_pixel_y, current_pose.x, current_pose.y, false);
        XYToPixel(past_pixel_x, past_pixel_y, past_pose.x, past_pose.y, false);

        theta = (current_pose.theta - past_pose.theta)/180 * M_PI
        trans_x = current_pixel_x - past_pixel_x;
        trans_y = current_pixel_y - past_pixel_y;
        
        Mat translationMatrix = (Mat_<double>(2,3) << 1, 0, trans_x, 0, 1, trans_y);
        Point2f rotation_center(map.cols/2, map.rows);
        Mat rotationMatrix = getRotationMatrix2D(rotation_center, theta, 1.0)
        
        map = warpAffine(map, map, translationMatrix, map.size());
        map = warpAffine(map, map, rotationMatrix, map.size());

    }

    void make_poses_list(slam::Data pose){
        if(poses_list.size() < 2) poses_list.push_back(pose);
        else{
            vector<slam::Data>::iterator it = poses_list.begin();
            it = poses_list.erase(it);
            poses_list.push_back(pose);
        }
    }

    // void make_costmaps_list(Mat costmap){
    //     if(costmaps_list.size() < 5) costmaps_list.push_back(costmap);
    //     else{
    //         vector<Mat>::iterator it = costmaps_list.begin();
    //         it = costmaps_list.erase(it);
    //         costmaps_list.push_back(costmap);
    //     }
    // }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_img_;
    ros::Subscriber sub_pose_;
    ros::Time mainclock;

    double gamma;
    slam::Data pose;
    slam::Data past_pose;
    slam::Data current_pose;
    Mat final_costmap;
    // vector<Mat> costmaps_list;
    vector<slam::Data> poses_list;
    
};




int main(int argc, char **argv){
    ros::init(argc, argv, "improved_costmap_publisher");
    ImprovedCostmap improved_costmap;
    while(ros::ok()){
        improved_costmap.quit_if_end();
        ros::spinOnce();
    }
    return 0;
}
