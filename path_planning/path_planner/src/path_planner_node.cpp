#include "rrt_star.h"
#include "defs.h"
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

extern Cor decision(const vector<geometry_msgs::PoseStamped> & goals, const vector<vector<double>> & costmap, int task, int light, int motion, int parking_space, bool & parking_complished_changed, bool & unparking_complished_changed);

class RosNode{
private:
	ros::NodeHandle n;
	ros::Subscriber cost_map_sub;
	ros::Subscriber mission_state_sub;	
	ros::Subscriber goals_sub;
	ros::Publisher path_pub;
	ros::Publisher gear_state_pub;

	int task, light, motion, parking_space;
	nav_msgs::Path goals;
public:
	double stepsize_pp_value;
	bool isTrackDriving;
	RosNode(){
		cost_map_sub = n.subscribe("cost_map_with_goal_vector", 5, &RosNode::costmapCallback, this);
		mission_state_sub = n.subscribe("mission_state", 50, &RosNode::missionstateCallback, this);
		goals_sub = n.subscribe("goals", 50000, &RosNode::goalsCallback, this);
		path_pub = n.advertise<nav_msgs::Path>("local_path", 1000);
		gear_state_pub = n.advertise<std_msgs::UInt32>("gear_state",10);

		task = light = motion = parking_space = -1;
		n.getParam("/isTrackDriving", isTrackDriving);
		n.getParam("/stepsize_pp", stepsize_pp_value);
	}

	void missionstateCallback(const std_msgs::UInt32 & msg){
		int mask = 0xF;
		int data = msg.data;
		motion = data & mask;
		light = (data>>4) & mask;
		task = (data>>8) & mask;
		//parking_space = (data>>12) & mask;
	}

	void goalsCallback(const nav_msgs::Path & msg){
		goals = msg;
	}

	void costmapCallback(const nav_msgs::OccupancyGrid & map){
		static int time_parking_complished{0};
		static int gear_state{0};

		cout<<"cost_map callback\n";
		if(isTrackDriving){
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
			rrt.solve(path,cost_map,x, y);

			cv::namedWindow("costmap_path");
			cv::Mat image(h,w,CV_8UC3);
			for(int i = 0;i<h;i++) for(int j = 0;j<w;j++){
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
				poseStamped.pose.position.x = cor.x-100;
				poseStamped.pose.position.y = cor.y;
				local_path.poses.push_back(poseStamped);
			}
			poseStamped.header.seq = 0;
			local_path.poses.push_back(poseStamped);

			// publish
			path_pub.publish(local_path);

			ROS_INFO("pub, duration : %ld",clock()-t);
		}
		else {
			if(goals.poses.empty()) return;
			if(task == -1) return;
			
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
			int t = clock();

			// get costmap	
			vector<vector<double>> cost_map(map.info.height,vector<double>(map.info.width));
			int h = map.info.height;
			int w = map.info.width;
			for(int i = 0; i<h; i++){
				for(int j = 0; j<w;j++){
					cost_map[i][j] = (double)(map.data[i*w+j]+ 1);
				}
			}

			// rrt star algorithm
			vector<Cor> path;
			Cor x(0,w/2);
			
			// 1. initial gear_state is forward(0)
			// 2. when parking_complished if true, time_parking_complished is recorded.
			// 3. when 20s is passed, gear_state is changed to rear(1) and time_parking_complished is reset to 0.
			// 4. finally, when unparking_complished is true, gear_state is changed to front(0).
			bool parking_complished_changed = false, unparking_complished_changed = false;
			Cor y = decision(goals.poses, cost_map, task, light, motion, parking_space, parking_complished_changed, unparking_complished_changed);
			
			if(parking_complished_changed){
				time_parking_complished = clock();
				// if  accidently clock() == 0
				if(time_parking_complished == 0) ++time_parking_complished;
			}
			//////////////////////////////////////////////////////////////////////
			// already stop but available path occur then stop  (because of localization error)
			else if(time_parking_complished != 0){
				y.x = 0; y.y = 0;
				if((clock() - time_parking_complished)/CLOCKS_PER_SEC > 20){
					gear_state = 1;
					time_parking_complished = 0;
				} 
			}
			else if(unparking_complished_changed){
				gear_state = 0;
			}

			// parking...  pub to slam team
			std_msgs::UInt32 msg;
			msg.data = gear_state;
			gear_state_pub.publish(msg);


			// stop!!!!!
			if(y.x == 0 && y.y == 0){
				nav_msgs::Path local_path;	
				path_pub.publish(local_path);
				return;
			}

			y.y+=w/2;
			
			//rrt.solve(path,cost_map,x, y, task == OBSTACLE_SUDDEN);

			for(Cor point :  path)
				cout << point.x << "," << point.y << endl;
			if(path.empty()){
				nav_msgs::Path local_path;	
				path_pub.publish(local_path);
				return;
			}

			// not sure
			cv::namedWindow("costmap_path");
			cv::Mat image(h,w,CV_8UC3);
			for(int i = 0;i<h;i++) for(int j = 0;j<w;j++){
				image.at<cv::Vec3b>(h-1-i,w-1-j)[0] = cost_map[i][j];	
				image.at<cv::Vec3b>(h-1-i,w-1-j)[1] = cost_map[i][j];	
				image.at<cv::Vec3b>(h-1-i,w-1-j)[2] = cost_map[i][j];	
			}
			for(int i = 0;i<path.size()-1;i++)
				line(image, cv::Point(w-1-path[i].y,h-1-path[i].x), cv::Point(w-1-path[i+1].y,h-1- path[i+1].x), cv::Scalar(100,200,50),1,0);

			cv::imshow("costmap_path",image);
			cv::waitKey(1);

			// convert path
			nav_msgs::Path local_path;


			// tracker use another coordinate system, heading = y axis!, right = x axis!
			geometry_msgs::PoseStamped poseStamped;
			int cnt = 0;
			for(Cor cor : path){
				poseStamped.header.seq = ++cnt;
				poseStamped.pose.position.x = - (cor.y-w/2);
				poseStamped.pose.position.y = cor.x;
				local_path.poses.push_back(poseStamped);
			}
			poseStamped.header.seq = 0;
			local_path.poses.push_back(poseStamped);

			local_path.header.seq = 0;
			if (motion == HALT_MOTION)
				local_path.header.seq |= 0x1;
			if (gear_state == 1)
				local_path.header.seq |= 0x10;

			// publish
			path_pub.publish(local_path);

			ROS_INFO("pub, duration : %ld",clock()-t);

		}
	}
};



int main(int argc, char **argv)
{
	ros::init(argc, argv, "path_planner");
	RosNode rosnode;
	ros::spin();
	return 0;
}
