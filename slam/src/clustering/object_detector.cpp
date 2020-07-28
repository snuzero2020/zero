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
#include "geometry_msgs/Point.h"
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;

class ObjectDetector{
    public:
    ObjectDetector(){
        pub_ = nh_.advertise<slam::Cluster>("/2d_obstacle_clouds", 10);
        sub_ = nh_.subscribe("/points", 1, &ObjectDetector::callback, this);
        iteration_ = 200;
        plane_tolerance_ = 0.05;
        cluster_tolerance_ = 0.10;
        cluster_threshold_ = 5;
        lidar_angle_ = 18;
        lidar_height_ = 1.25;
        zero_position_.x = 0.0;
        zero_position_.y = 0.0;
        zero_position_.z = 0.0;
        x_position_.x = 1.0;
        x_position_.y = 0.0;
        x_position_.z = 0.0;
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

    void select_candidate(){
        candidate_points_.clear();
        int index = 0;
        for(geometry_msgs::Point point : cloud_points_){
            if (-point.x*sin(lidar_angle_*M_PI/180)+point.z*cos(lidar_angle_*M_PI/180) < -lidar_height_ + plane_tolerance_*0){
                candidate_points_.push_back(index);
            }
            index++;
        }
        ROS_INFO("# of candidate points : %d",index);
    }

    void ransac_plane(){
        /*
        set<int> inliers_result;
        plane_config_[0] =0.0;
        plane_config_[1] =0.0;
        plane_config_[2] =0.0;
        plane_config_[3] =0.0;
        int iter = iteration_;
        while(iter>0){
            set<int> inliers;
            while(inliers.size()<3) inliers.insert(candidate_points_[rand()%candidate_points_.size()]);
            //check if they have same channel
            set<int>::iterator check_channels = inliers.begin();
            check_channels++;
            int ch1 = cloud_channels_[*check_channels];
            check_channels++;
            int ch2 = cloud_channels_[*check_channels];
            check_channels++;
            int ch3 = cloud_channels_[*check_channels];
            if(ch1==ch2 && ch2==ch3) continue;
            double x1,y1,z1,x2,y2,z2,x3,y3,z3;
            set<int>::iterator inliers_iter = inliers.begin();
            inliers_iter++;
            x1 = cloud_points_.at(*inliers_iter).x;
            y1 = cloud_points_.at(*inliers_iter).y;
            z1 = cloud_points_.at(*inliers_iter).z;
            inliers_iter++;
            x2 = cloud_points_.at(*inliers_iter).x;
            y2 = cloud_points_.at(*inliers_iter).y;
            z2 = cloud_points_.at(*inliers_iter).z;
            inliers_iter++;
            x3 = cloud_points_.at(*inliers_iter).x;
            y3 = cloud_points_.at(*inliers_iter).y;
            z3 = cloud_points_.at(*inliers_iter).z;
            double a,b,c,d;
            a = ((y2-y1)*(z3-z1) - (y3-y1)*(z2-z1));
            b = ((z2-z1)*(x3-x1) - (x2-x1)*(z3-z1));
            c = ((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1));
            d = -(a*x1 + b*y1 + c*z1);
            if(c<0) continue;
            double angle = acos(c/sqrt(a*a+b*b+c*c))*180/M_PI;
            if(angle < lidar_angle_ - 5.0 || angle > lidar_angle_ + 5.0) {
                iter--;
                continue;
            }
            int index = 0;
            for(geometry_msgs::Point point : cloud_points_){
                if(inliers.find(index) != inliers.end()) {
                    index ++;
                    continue;
                }
                double x4,y4,z4;
                x4 = cloud_points_[index].x;
                y4 = cloud_points_[index].y;
                z4 = cloud_points_[index].z;
                double dist = abs(a*x4+b*y4+c*z4+d)/sqrt(a*a+b*b+c*c);
                if (dist < plane_tolerance_) inliers.insert(index);
                index++;
            }
            if (inliers.size() > inliers_result.size()){
                inliers_result = inliers;
                plane_config_[0] = a;
                plane_config_[1] = b;
                plane_config_[2] = c;
                plane_config_[3] = d;
            }
            iter --;
        }
        */
        plane_config_[0]=-sin(18*M_PI/180);
        plane_config_[1]=0.0;
        plane_config_[2]=cos(18*M_PI/180);
        plane_config_[3]=-1.25;
        set<int> inliers_result;
        for(int i =0;i<cloud_points_.size();i++){
            geometry_msgs::Point point = cloud_points_.at(i);
            if (-point.x*sin(lidar_angle_*M_PI/180)+point.z*cos(lidar_angle_*M_PI/180) < -lidar_height_ + plane_tolerance_*4){
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
        cloud_points_ = msg->points;
        cloud_channels_ = msg->channels;
        candidate_points_.clear();
        filtered_points_.clear();
        filtered_channels_.clear();
        projected_points_.clear();
        clustering_helper_.clear();

        select_candidate();
        ransac_plane();
        projecting_points();
        vector<vector<int>> clusters = clustering();

        //Publish Data
        slam::Cluster rt;
        vector<geometry_msgs::Point> rt_points;
        vector<int> rt_channels;
        vector<int> rt_clusters;
        rt.header.stamp = ros::Time::now();
        rt.count = projected_points_.size();
        int cluster_index = 0;
        for(vector<int> cluster : clusters){
            for(int index : cluster){
                rt_points.push_back(projected_points_.at(index));
                rt_channels.push_back(filtered_channels_.at(index));
                rt_clusters.push_back(cluster_index);
            }
            cluster_index ++;
        }
        rt.points = rt_points;
        rt.channels = rt_channels;
        rt.clusters = rt_clusters;
        //rt.points = projected_points_;
        //rt.channels = filtered_channels_;
        pub_.publish(rt);
        clock_t end = clock();
        ROS_INFO("# of filtered points : %d", projected_points_.size());
        ROS_INFO("elapsed time : %lf",double(end-begin)/CLOCKS_PER_SEC);
        ROS_INFO("cosine value between z and normal : %lf", acos(plane_config_[2]
        /sqrt(plane_config_[0]*plane_config_[0]+plane_config_[1]*plane_config_[1]+plane_config_[2]*plane_config_[2]))*180/M_PI);
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    vector<geometry_msgs::Point> cloud_points_;
    vector<int> cloud_channels_;
    vector<int> candidate_points_;
    vector<geometry_msgs::Point> filtered_points_;
    vector<int> filtered_channels_;
    vector<geometry_msgs::Point> projected_points_;
    vector<int> clustering_helper_;
    int iteration_;
    double plane_config_[4];
    double lidar_angle_;
    double lidar_height_;
    double plane_tolerance_;
    double cluster_tolerance_;
    int cluster_threshold_;
    geometry_msgs::Point zero_position_;
    geometry_msgs::Point x_position_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "object_detector");
    ObjectDetector object_detector;
    ros::spin();
}
