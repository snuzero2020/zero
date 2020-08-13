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

class LidarPoseEstimator{
    public:
    LidarPoseEstimator(){
        sub_ = nh_.subscribe("/points", 1, &LidarPoseEstimator::callback, this);
        iteration_ = 200;
        plane_tolerance_ = 0.05;
        lidar_angle_ = 18;
        lidar_height_ = 1.25;
        count_ = 0;
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

    void select_candidate(){
        candidate_points_.clear();
        int index = 0;
        for(geometry_msgs::Point point : cloud_points_){
            if (-point.x*sin(lidar_angle_*M_PI/180)+point.z*cos(lidar_angle_*M_PI/180) < -lidar_height_ + plane_tolerance_*1){
                candidate_points_.push_back(index);
            }
            index++;
        }
    }

    void ransac_plane(){
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
        vector<bool> check_points;
        for(int i=0;i<cloud_points_.size();i++) check_points.push_back(false);
        for(int index : inliers_result) check_points.at(index) = true;
        for(int i=0;i<check_points.size();i++){
            if(check_points.at(i)) continue;
            filtered_points_.push_back(cloud_points_.at(i));
            filtered_channels_.push_back(cloud_channels_.at(i));
        }
        
    }


    
    void callback(const slam::Lidar::ConstPtr& msg){
        clock_t begin = clock();
        cloud_points_ = msg->points;
        cloud_channels_ = msg->channels;
        candidate_points_.clear();
        filtered_points_.clear();
        filtered_channels_.clear();
        
        select_candidate();
        ransac_plane();
        
        clock_t end = clock();
        
        count_ ++;
        theta_.push_back(acos(plane_config_[2]/sqrt(plane_config_[0]*plane_config_[0]+plane_config_[1]*plane_config_[1]+plane_config_[2]*plane_config_[2]))*180/M_PI);
        height_.push_back(plane_config_[3]/sqrt(plane_config_[0]*plane_config_[0]+plane_config_[1]*plane_config_[1]+plane_config_[2]*plane_config_[2]));
        ROS_INFO("cosine value between z and normal : %lf", acos(plane_config_[2]
        /sqrt(plane_config_[0]*plane_config_[0]+plane_config_[1]*plane_config_[1]+plane_config_[2]*plane_config_[2]))*180/M_PI);
        ROS_INFO("estimate height of lidar : %lf", plane_config_[3]/sqrt(plane_config_[0]*plane_config_[0]+plane_config_[1]*plane_config_[1]+plane_config_[2]*plane_config_[2]));
        if(count_ % 100 == 0){
            int n = count_ / 100;
            sort(theta_.begin(),theta_.end());
            sort(height_.begin(), height_.end());
            double sum_theta = 0, sum_height = 0;
            for(int i =10*n;i<90*n;i++){
                sum_theta += theta_.at(i);
                sum_height += height_.at(i);
            }
            sum_theta = sum_theta /(80*n);
            sum_height = sum_height /(80*n);

            ROS_INFO("Average theta : %.5lf, height : %.5lf (%5d step)", sum_theta, sum_height, count_);
        }
    }

    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    vector<geometry_msgs::Point> cloud_points_;
    vector<int> cloud_channels_;
    vector<int> candidate_points_;
    vector<geometry_msgs::Point> filtered_points_;
    vector<int> filtered_channels_;
    int iteration_;
    double plane_config_[4];
    double lidar_angle_;
    double lidar_height_;
    double plane_tolerance_;
    vector<double> theta_;
    vector<double> height_;
    int count_;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_pose_estimator");
    LidarPoseEstimator lidar_pose_estimator;
    ros::spin();
}
