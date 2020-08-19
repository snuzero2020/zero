#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "geometry_msgs/Point.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"


#include "slam/Cluster.h"
#include "slam/Clusters.h"
#include "slam/Lidar.h"
#include "slam/LidarPoint.h"
#include "slam/Data.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"

#include "Eigen/Eigen"
#include "XYToPixel.h"

using namespace std;
using namespace Eigen;

typedef pair<double, double> pdd;


class ObstacleCostMapPublisher{
    private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub_position;
    ros::Subscriber sub_lidar;
    pdd prv_position;
    pdd cur_position;
    double prv_heading;
    double cur_heading;
    bool first_subscriber;
    cv::Mat obstacle_cost_map;
    int width;
    int height;
    double resolution = 0.03;

    public:
    ObstacleCostMapPublisher(){
        pub = nh.advertise<sensor_msgs::Image>("/obstacle_cost_map", 2);
        sub_position = nh.subscribe("/fitered_data", 2, &ObstacleCostMapPublisher::callback_position, this);
        sub_lidar = nh.subscribe("/point_cloud_clusters", 2, &ObstacleCostMapPublisher::callback_lidar, this);
        first_subscriber = true;
        width = 300;
        height = 300;
        obstacle_cost_map = cv::Mat(width, height, CV_8UC1, cv::Scalar(0));
    }    
    
    void callback_position(const slam::Data::ConstPtr& msg){
        cur_position.first = msg->x;
        cur_position.second = msg->y;
        cur_heading = msg->theta;
    }

    void callback_lidar(const slam::Clusters::ConstPtr &msg){
        pdd position = cur_position;
        double heading = cur_heading;
        

        if(first_subscriber){
            cv::Mat cur_obstacle_cost_map = cv::Mat(width, height, CV_8UC1, cv::Scalar(0));
            for(slam::Cluster cluster : msg->clusters){
                for(slam::LidarPoint point : cluster.points){
                    int pixel_x = int(height-point.point_2d.x / resolution);
                    int pixel_y = int(width/2 + point.point_2d.y / resolution);
                    if(pixel_x<0 || pixel_x>=height || pixel_y<0 || pixel_y>=width) continue;
                    cur_obstacle_cost_map.at<uchar>(pixel_x, pixel_y) = 255;
                }
            }
            obstacle_cost_map = cur_obstacle_cost_map;
            //first_subscriber = false;

        }

        cv_bridge::CvImage img_bridge;
		sensor_msgs::Image img_msg;
		std_msgs::Header header;
		img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, obstacle_cost_map);
		img_bridge.toImageMsg(img_msg);
		pub.publish(img_msg);

        prv_position = position;
        prv_heading = heading;
    }



};

int main(int argc, char **argv){
    ros::init(argc, argv, "obstacle_cost_map_publisher");
    ObstacleCostMapPublisher obstacle_cost_map_publisher;
    ros::spin();
}
