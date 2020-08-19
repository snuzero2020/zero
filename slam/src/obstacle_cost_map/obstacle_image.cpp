#include <iostream>
#include <math.h>
#include <set>
#include <stdlib.h>
#include <string>
#include <vector>

#include "slam/Cluster.h"
#include "slam/LidarPoint.h"
#include "slam/Clusters.h"
#include "slam/Lidar.h"

#include "geometry_msgs/Point.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;
using namespace cv;


class ObstacleImage{
    public:
    dilation_element = getStructuringElement(MORPH_ELLIPSE, Size(18,18), Point(-1,-1));

	ObstacleImage(){
        pub_ = nh_.advertise<sensor_msgs::Image>("/obstacle_map/image_raw", 10);
        sub_ = nh_.subscribe("/point_cloud_clusters", 1, &ObstacleImage::callback, this);
    }

    void callback(const slam::Clusters::ConstPtr& msg){
        clock_t begin = clock();
        Mat obstacle_map = Mat(height_, width_, CV_8UC3,Scalar(255,255,255));
        clusters_ = msg -> clusters;
        for(slam::Cluster cluster_iter : clusters_){
            vector<slam::LidarPoint> points_ = cluster_iter.points;
            for(slam::LidarPoint point_iter : points_){
                int x = (int)(point_iter.point_2d.x/resolution_);
                int y = (int)(point_iter.point_2d.y/resolution_) + width_/2;
                if (x<0 || x>=height_ || y<0 || y>=width_) continue;
                Vec3b& color = obstacle_map.at<Vec3b>(x,y);
                color[0] = 0; color[1] = 0; color[2] = 0;
            }
        }
		dilate(obstacle_map, obstacle_map, dilation_element);
        sensor_msgs::Image rt;
        std_msgs::Header header;
        header.seq = msg->header.seq; // user defined counter
        header.stamp = ros::Time::now(); // time
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, obstacle_map);
        img_bridge.toImageMsg(rt); // from cv_bridge to sensor_msgs::Image
        pub_.publish(rt); // ros::Publisher pub_img = node.advertise<sensor_msgs::Image>("topic", queuesize);
        clock_t end = clock();
        ROS_INFO("elapsed time : %lf", double(end-begin)/CLOCKS_PER_SEC);
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    int height_ = 300;
    int width_ = 300;
    double resolution_ = 0.03;
    vector<slam::Cluster> clusters_;
    cv_bridge::CvImage img_bridge;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "obstacle_image");
    ObstacleImage obstacle_image;
    ros::spin();
}
