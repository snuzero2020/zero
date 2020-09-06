
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <set>
#include <string>
#include <vector>
#include "opencv2/opencv.hpp"

#include "std_msgs/Int32.h"
#include "std_msgs/Int32MultiArray.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point.h"

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

class ObstacleDetector{
    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_count_;
    ros::Subscriber sub_lidar_;
    ros::Subscriber sub_position_;
    ros::Subscriber sub_imu;
    vector<slam::LidarPoint> cloud_points_;
    vector<slam::LidarPoint> filtered_points_;
    vector<int> clustering_helper_;
    pdd current_position_;
    double current_heading_;
    double lidar_angle_;
    double lidar_height_;
    double plane_config_[4]; 
    double plane_tolerance_;
    double cluster_tolerance_;
    int cluster_threshold_;
    stringstream path_stream_;
    cv::Mat road_map_;
    double max_time = 0.0;
    double pitch_offset;

    public:
    bool is_kcity;
    ObstacleDetector(){
	ros::param::get("/is_kcity",is_kcity);
        pub_ = nh_.advertise<slam::Clusters>("/point_cloud_clusters", 2);
	    pub_count_ = nh_.advertise<std_msgs::Int32MultiArray>("cluster_count",2);
        sub_lidar_ = nh_.subscribe("/points", 1, &ObstacleDetector::callback_lidar, this);
        sub_imu = nh_.subscribe("/imu_raw", 1, &ObstacleDetector::callback_imu, this);
        sub_position_ = nh_.subscribe("/filtered_data", 1, &ObstacleDetector::callback_position, this);
        sub_mission_ = nh_.subscribe("/mission_state", 1, &ObstacleDetector::callback_mission, this);
        current_position_.first = 0.0;
        current_position_.second = 0.0;
        current_heading_ = 0.0;
        lidar_angle_ = 18.48311;
        lidar_height_ = 1.164920;
        plane_tolerance_ = 0.15;
        cluster_tolerance_ = 0.10;
        cluster_threshold_ = 5;
        mission_state_ = -1;
        if(is_kcity) path_stream_ << ros::package::getPath("slam") << "/config/KCity/KCity_road_area_eroded.png";
        else path_stream_ << ros::package::getPath("slam")<<"/config/FMTC/FMTC_road_area_eroded.png";
        
        road_map_ = cv::imread(path_stream_.str());
        cout << road_map_.size() << endl;
    }

    double get_distance(slam::LidarPoint p1, slam::LidarPoint p2){
        return sqrt((p1.point_2d.x-p2.point_2d.x)*(p1.point_2d.x-p2.point_2d.x)+(p1.point_2d.y-p2.point_2d.y)*(p1.point_2d.y-p2.point_2d.y));
    }

    inline int find(int a){
        if(clustering_helper_[a]<0) return a;
        clustering_helper_[a] = find(clustering_helper_[a]);
        return clustering_helper_[a];
    }

    inline void merge(int a, int b){
        a = find(a);
        b = find(b);
        if(a==b) return;
        clustering_helper_[b] = a;
    }
    
    void removing_plane(){
        set<int> inliers_result;
        pitch_offset = 0.0;
        plane_config_[0]=-sin(lidar_angle_*M_PI/180 - pitch_offset);
        plane_config_[1]=0.0;
        plane_config_[2]=cos(lidar_angle_*M_PI/180 - pitch_offset);
        plane_config_[3]=lidar_height_;

        cout << "normal vector: " << plane_config_[0] << endl;
        int n = cloud_points_.size();
        for(int i = 0; i<n;i++){
            geometry_msgs::Point p = cloud_points_.at(i).point_3d;

            if (p.x*plane_config_[0]+p.y*plane_config_[1]+p.z*plane_config_[2] + plane_config_[3] < plane_tolerance_){
                inliers_result.insert(i);
            }
        }
        ROS_INFO_STREAM("# of inliers: " << inliers_result.size());
        vector<bool> check_points;
        for(int i=0;i<cloud_points_.size();i++) check_points.push_back(false);
        for(int index : inliers_result) check_points.at(index) = true;
        for(int i=0;i<check_points.size();i++){
            if(check_points.at(i)) continue;
            filtered_points_.push_back(cloud_points_.at(i));
        }   
    }

    void removing_offroad(){
        vector<slam::LidarPoint> in_road;
        int pixel_x, pixel_y;
        for(slam::LidarPoint point : filtered_points_){
            double x = current_position_.first + point.point_2d.x*cos(current_heading_) - point.point_2d.y*sin(current_heading_);
            double y = current_position_.second + point.point_2d.x*sin(current_heading_) + point.point_2d.y*cos(current_heading_);
            XYToPixel(pixel_x, pixel_y, x, y, is_kcity);
            if(pixel_x > road_map_.cols || pixel_x < 0 || pixel_y > road_map_.rows || pixel_y < 0){
                ROS_INFO("A point is out of the map : (%5d, %5d)", pixel_x, pixel_y);
                continue;
            }
            cv::Vec3b color = road_map_.at<cv::Vec3b>(pixel_y, pixel_x);
            if(color[0]==0 && color[1] == 0 && color[2]==0) continue; // (x,y) is off-road point
            in_road.push_back(point);
            clustering_helper_.push_back(-1);
        }
        filtered_points_ = in_road;
    }

    vector<vector<int>> clustering(){
        int n = filtered_points_.size();
        for(int i = 0;i<n;i++){
            for(int j=i+1;j<n;j++){
                if(find(i) == find(j)) continue;
                if(get_distance(filtered_points_.at(i), filtered_points_.at(j))<cluster_tolerance_){
                    merge(i,j);
                }
            }
        }
        vector<vector<int>> storage;
        vector<vector<int>> clusters;
        for(int i=0;i<n;i++) {
            vector<int> cluster;
            storage.push_back(cluster);
        }
        for(int i=0;i<n;i++){
            storage.at(find(i)).push_back(i);
        }
        for(int i=0;i<n;i++){
            if(storage.at(i).size()<cluster_threshold_) continue;
            clusters.push_back(storage.at(i));
        }
        return clusters;
    }

    void callback_imu(const std_msgs::Float32::ConstPtr& msg){
        pitch_offset = msg->data;
        cout << pitch_offset << endl;
    }

    void callback_position(const slam::Data::ConstPtr& msg){
        current_position_.first = msg->x;
        current_position_.second = msg->y;
        current_heading_ = msg->theta;
    }

    void callback_mission(const std_msgs::UInt32::ConstPtr& msg){
        mission_state_ = msg->data;
    }

    void callback_lidar(const slam::Lidar::ConstPtr& msg){
        clock_t begin = clock();
	    cloud_points_ = msg->points;
        filtered_points_.clear();
        clustering_helper_.clear();

        removing_plane();
        removing_offroad();
        vector<vector<int>> clusters = clustering();

        //Publish Data
        slam::Clusters rt;
	std_msgs::Int32MultiArray rt_count;
	vector<int> cluster_counts;
        vector<slam::Cluster> rt_clusters;
        vector<geometry_msgs::Point> rt_points;
        rt.header.stamp = ros::Time::now();

        for(vector<int> cluster : clusters){
            slam::Cluster rt_cluster;
            rt_cluster.count = 0;

            for(int index : cluster){
                rt_cluster.points.push_back(filtered_points_.at(index));
                rt_cluster.count ++;
            }
            rt_clusters.push_back(rt_cluster);
	    cluster_counts.push_back(rt_cluster.count);
        }
        rt.clusters = rt_clusters;
	rt_count.data = cluster_counts;
        pub_.publish(rt);
	pub_count_.publish(rt_count);
        clock_t end = clock();
        if(double(end-begin)/CLOCKS_PER_SEC > max_time) max_time = double(end-begin)/CLOCKS_PER_SEC;

        ROS_INFO_STREAM("# of filtered points : " << filtered_points_.size());
        ROS_INFO_STREAM("elapsed time : " << double(end-begin)/CLOCKS_PER_SEC);
        //ROS_INFO_STREAM("cosine value between z and normal : " << acos(plane_config_[2]
        ///sqrt(plane_config_[0]*plane_config_[0]+plane_config_[1]*plane_config_[1]+plane_config_[2]*plane_config_[2]))*180/M_PI);
        //ROS_INFO_STREAM("max time : " << max_time);
	    //ROS_INFO("plane coefficient : %lf", plane_coefficient_);
    }

};
le_ = 18.48311;
        lidar_height_ = 1.164920;ctor");
    ObstacleDetector obstacle_detector;
    ros::spin();
}
