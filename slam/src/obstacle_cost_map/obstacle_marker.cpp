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

#include "XYToPixel.h"

using namespace std;

typedef pair<double, double> pdd;


class ObstacleMarker{
    private:
    ros::NodeHandle nh;
    ros::Subscriber sub_position;
    ros::Subscriber sub_lidar;
    pdd cur_position;
    double cur_heading;
    stringstream path_stream;
    stringstream save_stream;
    cv::Mat global_map;
    int count;

    public:
    ObstacleMarker(){
        sub_position = nh.subscribe("/filtered_data", 2, &ObstacleMarker::callback_position, this);
        sub_lidar = nh.subscribe("/point_cloud_clusters", 2, &ObstacleMarker::callback_lidar, this);
        path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_map.png";
        save_stream << ros::package::getPath("slam") << "/config/obstacle_marker.png";
        global_map = cv::imread(path_stream.str(),1);
        count = 0;
        if(global_map.empty()) ROS_ERROR("Map image is empty. Check your path");
        else ROS_INFO("Completed to load global map");
    }    
    
    void callback_position(const slam::Data::ConstPtr& msg){
        cur_position.first = msg->x;
        cur_position.second = msg->y;
        cur_heading = msg->theta;
    }

    void callback_lidar(const slam::Clusters::ConstPtr &msg){
        pdd position = cur_position;
        double heading = cur_heading;
                
        
        for(slam::Cluster cluster : msg->clusters){
            for(slam::LidarPoint point : cluster.points){
                double x = position.first + point.point_2d.x*cos(heading) - point.point_2d.y*sin(heading);
                double y = position.second + point.point_2d.x*sin(heading) + point.point_2d.y*cos(heading);
                int pixel_x, pixel_y;
                XYToPixel(pixel_x,pixel_y,x,y,false);
                cv::circle(global_map, cv::Point(pixel_x, pixel_y), 3, cv::Scalar(255,0,0), 1);
                //global_map.at<cv::Vec3b>(pixel_x, pixel_y)[0] = 255;
                //global_map.at<cv::Vec3b>(pixel_x, pixel_y)[1] = 0;
                //global_map.at<cv::Vec3b>(pixel_x, pixel_y)[2] = 0;
            }
        }

        if(count == 400) cv::imwrite(save_stream.str(), global_map);
        count ++;
    }


};

int main(int argc, char **argv){
    ros::init(argc, argv, "obstacle_marker");
    ObstacleMarker obstacle_marker;
    ros::spin();
}
