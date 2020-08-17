#include "ros/ros.h"
#include "ros/time.h"
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <math.h>
#include <set>
#include "slam/Lidar.h"
#include "slam/Cluster.h"
#include "slam/Clusters.h"
#include "geometry_msgs/Point.h"
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

class ObjectDetector{
    public:
    ObjectDetector(){
        pub_ = nh_.advertise<slam::Clusters>("/point_cloud_clusters", 10);
        sub_ = nh_.subscribe("/points", 1, &ObjectDetector::callback, this);
        plane_tolerance_ = 0.10;
        cluster_tolerance_ = 0.10;
        cluster_threshold_ = 5;
        lidar_angle_ = 18.97246;
        lidar_height_ = 1.24935;
        zero_position_.x = 0.0;
        zero_position_.y = 0.0;
        zero_position_.z = 0.0;
        x_position_.x = 1.0;
        x_position_.y = 0.0;
        x_position_.z = 0.0;
	    plane_coefficient_ = 0.0;
    }

    geometry_msgs::Point projection(geometry_msgs::Point point){
        geometry_msgs::Point rt;
        double t = (plane_config_[0]*point.x+plane_config_[1]*point.y+plane_config_[2]*point.z+plane_config_[3])
        /(plane_config_[0]*plane_config_[0]+plane_config_[1]*plane_config_[1]+plane_config_[2]*plane_config_[2]);
        rt.x = point.x - plane_config_[0]*t;
        rt.y = point.y - plane_config_[1]*t;
        rt.z = point.z - plane_config_[2]*t;
        return rt;
    }

    geometry_msgs::Point cross_product(geometry_msgs::Point p1, geometry_msgs::Point p2){
        geometry_msgs::Point rt;
        rt.x = p1.y*p2.z - p1.z*p2.y;
        rt.y = p1.z*p2.x - p1.x*p2.z;
        rt.z = p1.x*p2.y - p1.y*p1.x;
        return rt;
    }

    double dot_product(geometry_msgs::Point p1, geometry_msgs::Point p2){
        return p1.x*p2.x + p1.y*p2.y + p1.z*p2.z;
    }

    geometry_msgs::Point normalization(geometry_msgs::Point v){
        double t = sqrt(v.x*v.x+v.y*v.y+v.z*v.z);
        v.x = v.x/t;
        v.y = v.y/t;
        v.z = v.z/t;
        return v;
    }

    inline double get_distance(geometry_msgs::Point p1, geometry_msgs::Point p2){
        return sqrt((p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z));
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

    void ransac_plane(){
        plane_config_[0]=-sin(lidar_angle_*M_PI/180);
        plane_config_[1]=0.0;
        plane_config_[2]=cos(lidar_angle_*M_PI/180);
        plane_config_[3]=lidar_height_;
        set<int> inliers_result;
        for(int i =0;i<cloud_points_.size();i++){
            geometry_msgs::Point point = cloud_points_.at(i);
            if (abs(point.x*plane_config_[0]+point.z*plane_config_[2] + plane_config_[3]) < plane_coefficient_*plane_tolerance_){
                inliers_result.insert(i);
            }
        }
        ROS_INFO("# of inliers : %d", inliers_result.size());
        vector<bool> check_points;
        for(int i=0;i<cloud_points_.size();i++) check_points.push_back(false);
        for(int index : inliers_result) check_points.at(index) = true;
        for(int i=0;i<check_points.size();i++){
            if(check_points.at(i)) continue;
            filtered_points_.push_back(cloud_points_.at(i));
            filtered_channels_.push_back(cloud_channels_.at(i));
            clustering_helper_.push_back(-1);
        }
        
    }


    void projecting_points(){
        geometry_msgs::Point normal_vector;
        normal_vector.x = plane_config_[0];
        normal_vector.y = plane_config_[1];
        normal_vector.z = plane_config_[2];
        geometry_msgs::Point lidar_position = projection(zero_position_);
        geometry_msgs::Point lidar_x = projection(x_position_);
        geometry_msgs::Point lidar_y = cross_product(normal_vector,lidar_x);
        lidar_x = normalization(lidar_x);
        lidar_y = normalization(lidar_y);
        /*
        Matrix<double,3,2> A;
        A<<lidar_x.x,lidar_y.x, lidar_x.y,lidar_y.y, lidar_x.z,lidar_y.z;
        for(geometry_msgs::Point point : filtered_points_){
            Matrix<double,3,1> b;
            geometry_msgs::Point projected_point = projection(point);
            b <<projected_point.x-lidar_position.x, projected_point.y-lidar_position.y, projected_point.z-lidar_position.z;
            Matrix<double,2,1> solution = A.bdcSvd(ComputeThinU|ComputeThinV).solve(b);
            geometry_msgs::Point rt;
            rt.x = solution(0);
            rt.y = solution(1);
            rt.z = 0;
            projected_points_.push_back(rt);
        }
        */
        for(geometry_msgs::Point point : filtered_points_){
            geometry_msgs::Point projected_point = projection(point);
            projected_point.x -= lidar_position.x;
            projected_point.y -= lidar_position.y;
            projected_point.z -= lidar_position.z;
            
            geometry_msgs::Point rt;
            rt.x = dot_product(lidar_x, projected_point);
            rt.y = dot_product(lidar_y, projected_point);
            rt.z = 0;
            projected_points_.push_back(rt);
        }
    }

    vector<vector<int>> clustering(){
        int n = projected_points_.size();
        for(int i = 0;i<n;i++){
            for(int j=i+1;j<n;j++){
                if(find(i) == find(j)) continue;
                if(get_distance(projected_points_.at(i),projected_points_.at(j))<cluster_tolerance_){
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

    void callback(const slam::Lidar::ConstPtr& msg){
        clock_t begin = clock();
	    plane_coefficient_ = 1.1; // HARD CODING
        cloud_points_ = msg->points;
        cloud_channels_ = msg->channels;
        filtered_points_.clear();
        filtered_channels_.clear();
        projected_points_.clear();
        clustering_helper_.clear();

        ransac_plane();
        projecting_points();
        vector<vector<int>> clusters = clustering();

        //Publish Data
        slam::Clusters rt;
        vector<slam::Cluster> rt_clusters;
        vector<geometry_msgs::Point> rt_points;
        rt.header.stamp = ros::Time::now();
        for(vector<int> cluster : clusters){
            slam::Cluster rt_cluster;
            rt_cluster.count = 0;
            for(int index : cluster){
                rt_cluster.points_2d.push_back(projected_points_.at(index));
                rt_cluster.points_3d.push_back(filtered_points_.at(index));
                rt_cluster.count ++;
            }
            rt_clusters.push_back(rt_cluster);
        }
        rt.clusters = rt_clusters;
        pub_.publish(rt);
        clock_t end = clock();
        ROS_INFO("# of filtered points : %d", projected_points_.size());
        ROS_INFO("elapsed time : %lf",double(end-begin)/CLOCKS_PER_SEC);
        ROS_INFO("cosine value between z and normal : %lf", acos(plane_config_[2]
        /sqrt(plane_config_[0]*plane_config_[0]+plane_config_[1]*plane_config_[1]+plane_config_[2]*plane_config_[2]))*180/M_PI);
	//ROS_INFO("plane coefficient : %lf", plane_coefficient_);
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    vector<geometry_msgs::Point> cloud_points_;
    vector<int> cloud_channels_;
    vector<geometry_msgs::Point> filtered_points_;
    vector<int> filtered_channels_;
    vector<geometry_msgs::Point> projected_points_;
    vector<int> clustering_helper_;
    double plane_config_[4]; // 
    double lidar_angle_;
    double lidar_height_;
    double plane_tolerance_;
    double cluster_tolerance_;
    int cluster_threshold_;
    geometry_msgs::Point zero_position_;
    geometry_msgs::Point x_position_;
    double plane_coefficient_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "object_detector");
    ObjectDetector object_detector;
    ros::spin();
}
