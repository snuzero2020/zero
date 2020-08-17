#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_msgs/UInt32.h"

#include "slam/Data.h"
#include "slam/GlobalPathPoint.h"
#include "slam/Imu.h"

#include "ros/package.h"
#include "ros/ros.h"
#include "ros/time.h"

#include "UnixtimeToSec.h"


using namespace std;

class LocalPathPublisher{
    private:
    ros::NodeHandle nh;
    ros::Publisher local_path_pub;
    ros::Subscriber filter_data_sub;
    ros::Subscriber gear_state_sub;
	stringstream path_stream;

    slam::Data current_pose;

    double length{9.0};
    int pixel{300};

    vector<slam::GlobalPathPoint> global_path_; 
    const char delimiter_ = ' '; 

    //string input_file_ = "/home/snuzero/catkin_ws/src/zero/slam/src/global_path/global_path.txt";

    public:
    
	int gear_state{0};

    LocalPathPublisher(){
        local_path_pub = nh.advertise<nav_msgs::Path>("/goals", 2);
        gear_state_sub = nh.subscribe("/gear_state", 2, &LocalPathPublisher::gs_callback, this);
		filter_data_sub = nh.subscribe("/filtered_data", 2, &LocalPathPublisher::filter_data_callback, this);
        path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_global_path.txt";
        load_global_path();
    }
    
	void gs_callback(const std_msgs::UInt32 state){
		gear_state = state.data;
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
        while(getline(in, in_line)){
            stringstream ss(in_line);
            string token;
            vector<string> result;
            while(getline(ss, token, delimiter_)){
                result.push_back(token);
            }
            slam::GlobalPathPoint point;
	    if (result.size() == 0) break;
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
        if(!gear_state){
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
		}
		else{
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
            

				if(pose_change.pose.position.x<-1.05 && pose_change.pose.position.x>-1.05-length && pose_change.pose.position.y > 0 && pose_change.pose.position.y < length){
                	pose_change.pose.position.x = int((-pose_change.pose.position.x-1.05)/length*pixel);
                	pose_change.pose.position.y = (-1) * (int(pose_change.pose.position.y/length*pixel) - pixel/2);
                	local_path.poses.push_back(pose_change);
            	}
        	}
		}
	    cout << "# of local path: " << local_path.poses.size() << endl;
        local_path_pub.publish(local_path);
    }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "local_path_publisher");
    LocalPathPublisher local_path_publisher;
    ros::spin();
    return 0;
}

