#include "rrt_star.h"
#include <vector>
#include <ctime>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "core_msgs/Control.h"
#include <opencv2/opencv.hpp>
#include <iostream>

extern Cor decision(const vector<geometry_msgs::PoseStamped> & goals, const vector<vector<double>> & costmap, int task, int light, int motion, int x, int y, double angle);

class RosNode{
private:
	ros::NodeHandle n;
	ros::Subscriber cost_map_sub;
	ros::Subscriber mission_state_sub;	
	ros::Subscriber goals_sub;
	ros::Publisher path_pub;
	ros::Publisher estop_pub;
	int task, light, motion;
	nav_msgs::Path goals;
public:
	double start_point_x_value;
	double start_point_y_value;
	double goal_point_x_value;
	double goal_point_y_value;
	double stepsize_pp_value;
	RosNode(){
		cost_map_sub = n.subscribe("cost_map_with_goal_vector", 5, &RosNode::costmapCallback, this);
		mission_state_sub = n.subscribe("mission_state", 50, &RosNode::missionstateCallback, this);
		goals_sub = n.subscribe("goals", 50000, &RosNode::goalsCallback, this);
		path_pub = n.advertise<nav_msgs::Path>("local_path", 1000);
		estop_pub = n.advertise<core_msgs::Control>("car_signal", 10);
		task = light = motion = -1;
		n.getParam("/start_point_x", start_point_x_value);
		n.getParam("/start_point_y", start_point_y_value);
		n.getParam("/goal_point_x", goal_point_x_value);
		n.getParam("/goal_point_y", goal_point_y_value);
		n.getParam("/stepsize_pp", stepsize_pp_value);
	}

	void missionstateCallback(const std_msgs::UInt32 & msg){
		int mask = 0xF;
		int data = msg.data;
		motion = data & mask;
		light = (data>>4) & mask;
		task = (data>>8) & mask;
	}

	void goalsCallback(const nav_msgs::Path & msg){
		goals = msg;
	}

	void costmapCallback(const nav_msgs::OccupancyGrid & map){

////////////////with goal decision and mission recognizor
//		ROS_INFO("callback");
//		//if(goals.poses.empty()) return;
//		//if(task == -1) return;
//		static RRT rrt = RRT();
//		int t = clock();
//
//		// get costmap	
//		vector<vector<double>> cost_map(map.info.height,vector<double>(map.info.width));
//		int h = map.info.height;
//		int w = map.info.width;
//		for(int i = 0; i<h; i++){
//			for(int j = 0; j<w;j++){
//				cost_map[i][j] = (double)(map.data[i*w+j]);
//			}
//		}
//
//		// rrt star algorithm
//		vector<Cor> path;
//		Cor x(100,0);
//		Cor y(100,199);
//		//Cor y = decision(goals.poses, cost_map, task, light, motion, 100, 0, 0);
//		
//		// stop!!!!!
//		if(y.x < 0){
//			core_msgs::Control msg;
//			msg.is_auto = 1;
//			msg.estop = 0;
//			msg.gear = 0;
//			msg.brake = 50;
//			msg.speed = 0;
//			msg.steer = 0;
//			estop_pub.publish(msg);
//			return;
//		}
//
		// for track driving mission
		cout<<"callback\n";
		int iternum;
		double radius;
		double stepsize;
		double threshold;
		double threshold2;
		n.getParam("/iternum", iternum);
		n.getParam("/radius", radius);
		n.getParam("/stepsize_rrt", stepsize);
		n.getParam("/threshold", threshold);
		n.getParam("/threshold2", threshold2);		
		RRT rrt = RRT(iternum, radius, stepsize, threshold, threshold2);
		//rrt.print_RRT();
                int t = clock();
		cout << "map time stamp : " << map.header.stamp.sec << endl;
                // get costmap  
                vector<vector<double>> cost_map(map.info.height,vector<double>(map.info.width));
                int h = map.info.height;
                int w = map.info.width;
                for(int i = 0; i<h; i++){
                        for(int j = 0; j<w;j++){
                                cost_map[199-i][199-j] = ((double)map.data[j*w+i] * 4 * 100 / 255.0 ) + 1;// yellow line is 255 & white is 128
                        }
                }
                cout << "90,198 cost ; " << cost_map[90][198] << endl;

                // rrt star algorithm
                vector<Cor> path;
                Cor x(100,0), y(100,199);
                y.x = map.data[w*h]*2;
                y.y = map.data[w*h + 1]*2;
                std::cout << y.x << "," << y.y << std::endl;
		rrt.solve(path,cost_map,x, y, iternum);

		cv::namedWindow("costmap_path");
		cv::Mat image(rrt.map_length,rrt.map_length,CV_8UC3);
		for(int i = 0;i<rrt.map_length;i++) for(int j = 0;j<rrt.map_length;j++){
			image.at<cv::Vec3b>(i,j)[0] = cost_map[j][199-i];	
			image.at<cv::Vec3b>(i,j)[1] = cost_map[j][199-i];	
			image.at<cv::Vec3b>(i,j)[2] = cost_map[j][199-i];	
		}
		for(int i = 0;i<path.size()-1;i++)
			 line(image, cv::Point(path[i].x,199-path[i].y), cv::Point(path[i+1].x,199- path[i+1].y), cv::Scalar(100,200,50),1,0);

		cv::imshow("costmap_path",image);
		cv::waitKey(1);

		// convert path
		nav_msgs::Path local_path;
	
		geometry_msgs::PoseStamped poseStamped;
		int cnt = 0;
		for(Cor cor : path){
			poseStamped.header.seq = ++cnt;
			poseStamped.pose.position.x = cor.x;
			poseStamped.pose.position.y = cor.y;
			local_path.poses.push_back(poseStamped);
		}
		poseStamped.header.seq = 0;
		local_path.poses.push_back(poseStamped);

		// publish
		path_pub.publish(local_path);

		ROS_INFO("pub, duration : %ld",clock()-t);
	}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_planner");
	RosNode rosnode;
	ros::spin();
	return 0;
}
