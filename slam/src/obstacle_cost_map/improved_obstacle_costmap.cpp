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
        pub_ = nh_.advertise<sensor_msgs::Image>("obstacle_map/costmap",10);
        sub_img_ = nh_.subscribe("/obstacle_map/costmap", 1, &DecayingCostmap::img_callback, this);
        sub_pose_ = nh_.subscribe("/filtered_data", 1, &DecayingCostmap::pose_callback, this);
        final_costmap = Mat::zeros(300, 300, CV_8U);
        alpha = (sqrt(5) + 1) / 4; // decay_rate of current costmap
        beta = (sqrt(5) -1 ) / 4; // decay_rate of past costmap
    }

    //USE VECTOR'S ITERATION    
    void pose_callback(const slam::Data::ConstPtr& msg){ 
        pose.x = msg->x;
        pose.y = msg->y;
        pose.theta = msg->theta;
    }

    void img_callback(const sensor_msgs::Image::ConstPtr &msg){
        
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
            warpAffine(final_costmap, final_costmap, calibrate_map(poses_list[1], poses_list[0]), costmap.size());
            // Interpolate the influence of the consequential costmaps
            for(int i = 0 ; i < costmap.rows ; i++){
                for(int j = 0; j < costmap.cols; j++){
                    final_costmap.at<uchar>(j,i) = saturate_cast<uchar>(alpha * costmap.at<uchar>(j,i) + beta * final_costmap.at<uchar>(j,i));
                }
            }
        }

        imshow("original_costmap", costmap);
        imshow("improved_costmap", final_costmap);
        int key = waitKey(30);
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
        
        calibrationMatrix = T_current.inverse() * T_past;

        //2. Reference frame change between car's point and zero point in the image pixel world
        MatrixXd moveZeropoint(3,3);
        moveZeropoint << 0, 1, -150,
                        -1, 0, 300,
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
            destroyWindow("original_costmap");
            destroyWindow("improved_costmap");
        }
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_img_;
    ros::Subscriber sub_pose_;
    ros::Time mainclock;

    double alpha, beta;
    slam::Data pose;
    Mat final_costmap;
    Mat costmap;
    // vector<Mat> costmaps_list;
    vector<slam::Data> poses_list;
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
