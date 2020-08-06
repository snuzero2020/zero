#include "ros/ros.h"
#include "ros/time.h"
#include "slam/Imu.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Quaternion.h"
#include "slam/GlobalPathPoint.h"
#include "UnixtimeToSec.h"
#include "nav_msgs/Path.h"
#include "slam/Data.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <math.h>
#include <vector>
#include <cmath>
#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
using namespace std;

class LocalPathPublisher{
    
    private:
    ros::NodeHandle nh;
    ros::Publisher local_path_pub;
    ros::Subscriber filter_data_sub;

    slam::Data current_pose;

    double length{9.0};
    int pixel{300};

    vector<slam::GlobalPathPoint> global_path_; 
    const char delimiter_ = ' '; 

    string input_file_ = "/home/jeongwoooh/catkin_ws/a.txt";

    public:
    

    LocalPathPublisher(){
        local_path_pub = nh.advertise<nav_msgs::Path>("/goals", 2);
        filter_data_sub = nh.subscribe("/filtered_data", 2, &LocalPathPublisher::filter_data_callback, this);
        load_global_path();
    }
    
    void filter_data_callback(const slam::Data::ConstPtr& msg){
        current_pose.x = msg->x;
        current_pose.y = msg->y;
        current_pose.theta = msg->theta;
        global_to_local();
    } 

    void load_global_path(){
        string in_line;
        ifstream in(input_file_);
        while(getline(in, in_line)){
            stringstream ss(in_line);
            string token;
            vector<string> result;
            while(getline(ss, token, delimiter_)){
                result.push_back(token);
            }
            slam::GlobalPathPoint point;
            point.x = stod(result.at(0));
            point.y = stod(result.at(1));
            point.theta = stod(result.at(2));
            global_path_.push_back(point);
        }
    }

    void global_to_local(){
	    nav_msgs::Path local_path;
        slam::Data pose;
        geometry_msgs::PoseStamped pose_change;
        for (auto iter = global_path_.begin(); iter != global_path_.end(); iter++){

            pose.x = (*iter).x;
            pose.y = (*iter).y;
            pose.theta = (*iter).theta;

            double X = pose.x - current_pose.x - length/2.0 * sin(current_pose.theta);
            double Y = pose.y - current_pose.y + length/2.0 * cos(current_pose.theta);

            pose_change.pose.position.x = X * cos(current_pose.theta) + Y * sin(current_pose.theta);
            pose_change.pose.position.y = Y * cos(current_pose.theta) - X * sin(current_pose.theta);
            pose_change.pose.position.z = pose.theta - current_pose.theta;

            if(pose_change.pose.position.x>0 && pose_change.pose.position.x<length && pose_change.pose.position.y >0 && pose_change.pose.position.y<length){
                pose_change.pose.position.x = int(pose_change.pose.position.x/length*pixel);
                pose_change.pose.position.y = int(pose_change.pose.position.y/length*pixel) - pixel/2;
                local_path.poses.push_back(pose_change);   
            }
        }
	printf("# of local path : %d\n", local_path.poses.size());
        local_path_pub.publish (local_path);
    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "local_path_publisher");
    LocalPathPublisher localpathpublisher;
    ros::spin();
    return 0;
}
