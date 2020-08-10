#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"
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
#include<nav_msgs/OccupancyGrid.h>

using namespace std;

class LocalPathPublisher{
    private:
    ros::NodeHandle _nh;
    ros::Publisher _pub;
    ros::Subscriber _sub;
    
    stringstream _global_path_stream;
    vector<slam::GlobalPathPoint> _points;
    vector<vector<int>> _edges;
    int _count;
    
    double _max_distance = 9.0;
    int _max_pixel = 300;
    const char _delimiter = ' ';

    slam::Data _cur;
    int _cur_node;
    bool _is_far;

    public:
    LocalPathPublisher(){
        _pub = nh.advertise<nav_msgs::Path>("/goals", 2);
        _sub = nh.subscribe("/filtered_data", 2, &LocalPathPublisher::callback, this);
        _global_path_stream << ros::package::getPath("slam") << "/src/global_path/global_path_graph.txt";
        _is_far = true;
        load_global_path();
    }


    void load_global_path(){
        ifstream in(_global_path_stream.str());
        string in_line;
        stringstream count_info(in_line);
        string token;
        vector<string> result;
        getline(in, in_line);
        while(getline(count_info, token, delimiter_)) result.push_back(token);
        _count = stoi(result.at(0));
        for(int i = 0 ; i<_count;i++){
            getline(in, in_line);
            stringstream point_info(in_line);
            result.clear();
            while(getline(point_info, token, _delimiter)) result.push_back(token);
            slam::GlobalPathPoint point;
            point.x = stod(result.at(0));
            point.y = stod(result.at(1));
            point.theta = stod(result.at(2));
            point.flag = stod(result.at(3));
            _points.push_back(point);
            printf("\rloading global path points : %5d/%5d",i+1, _count);
        }
        for(int i=0;i<_count;i++){
            getline(in, in_line);
            stringstream edge_info(in_line);
            vector<int> edge;
            while(getline(edge_info,token,_delimiter)) edge.push_back(stoi(token));
            _edges.push_back(edge);
            printf("\rloading edges of the global path graph : %5d/%5d", i+1, _count);
        }
        printf("\ncompleted to load the edges of the global path graph : %5d\n",_count);
    }


    void callback(const slam::Data::ConstPtr& msg){
        _cur.x = msg->x;
        _cur.y = msg->y;
        _cur.theta = msg->theta;
        if(_is_far) {
            find_nearest();
            _is_far = false;
        }
    }
}



class LocalPathPublisher{
    private:
    ros::NodeHandle nh;
    ros::Publisher local_path_pub;
    ros::Subscriber filter_data_sub;
    stringstream path_stream;
    int count_ = 0;

    slam::Data current_pose;

    double length{9.0};
    int pixel{300};

    vector<slam::GlobalPathPoint> global_path_; 
    const char delimiter_ = ' '; 

    public:
    LocalPathPublisher(){
        local_path_pub = nh.advertise<nav_msgs::Path>("/goals", 2);
        filter_data_sub = nh.subscribe("/filtered_data", 2, &LocalPathPublisher::filter_data_callback, this);
        path_stream << ros::package::getPath("slam") << "/src/global_path/global_path_graph.txt";
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
        ifstream in(path_stream.str());
        stringstream ss(in_line);
        string token;
        vector<string> result;    
        getline(in, in_line);
        while(getline(ss, token, delimiter_)) result.push_back(token);
        count_ = stoi(result.at(0));
        for(int i = 0 ;i<count_;i++){
            getline(in, in_line);
            stringstream point_info(in_line);
            result.clear();
            while(getline(point_info, token, delimiter_)) result.push_back(token);
            slam::GlobalPathPoint point;
            point.x = stod(result.at(0));
            point.y = stod(result.at(1));
            point.theta = stod(result.at(2));
            point.flag = stod(result.at(3));
            global_path_.push_back(point);
            printf("\rloading global path points : %5d/%5d",i+1, count_);
        }
        printf("\ncompleted to load global path points : %5d\n", count_);
        for(int i = 0 ;i<count_;i++){
            getline(in, in_line);
            stringstream edge_info(in_line);
            result.clear();
        }
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
            point.flag = stod(result.at(3));
            cout << point.x << " " << point.y << " " << point.theta << " " << point.flag << endl; 
            global_path_.push_back(point);
        }
    }

    void global_to_local(){
	    nav_msgs::Path local_path;
        slam::GlobalPathPoint pose;
        geometry_msgs::PoseStamped pose_change;
        nav_msgs::OccupancyGrid local_goal;
        for (auto iter = global_path_.begin(); iter != global_path_.end(); iter++){

            pose.x = (*iter).x;
            pose.y = (*iter).y;
            pose.theta = (*iter).theta;
            pose.flag = (*iter).flag;

            double X = pose.x - current_pose.x - length/2.0 * sin(current_pose.theta);
            double Y = pose.y - current_pose.y + length/2.0 * cos(current_pose.theta);

            pose_change.pose.position.x = X * cos(current_pose.theta) + Y * sin(current_pose.theta);
            pose_change.pose.position.y = Y * cos(current_pose.theta) - X * sin(current_pose.theta);
            pose_change.pose.position.z = pose.theta - current_pose.theta;
            pose_change.header.seq = pose.flag;

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
    LocalPathPublisher local_path_publisher;
    ros::spin();
    return 0;
}
