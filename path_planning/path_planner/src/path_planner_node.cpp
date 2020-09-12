#include "rrt_star.h"
#include "defs.h"
#include <vector>
#include <ctime>
#include <cmath>
#include "ros/ros.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int32.h"
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

extern Cor decision(const vector<geometry_msgs::PoseStamped> & goals, const vector<vector<double>> & costmap, int task, int light, int motion, int parking_space, bool & parking_complished_changed, bool & unparking_complished_changed, int gear_state);

class RosNode{
private:
	ros::NodeHandle n;
	ros::Subscriber cost_map_sub;
	ros::Subscriber mission_state_sub;
	ros::Subscriber parking_space_sub;	
	ros::Subscriber goals_sub;
	ros::Publisher path_pub;
	ros::Publisher gear_state_pub;
	ros::Publisher parking_complished_pub;

	int task, light, motion, parking_space;
	nav_msgs::Path goals;
public:
	double stepsize_pp_value;
	int duration_parking{0};
	int duration_unparking{0};
	bool isTrackDriving;
	RosNode(){
		cost_map_sub = n.subscribe("cost_map_with_goal_vector", 2, &RosNode::costmapCallback, this);
		parking_space_sub = n.subscribe("parking_space", 2, &RosNode::parkingspaceCallback, this);
		mission_state_sub = n.subscribe("mission_state", 2, &RosNode::missionstateCallback, this);
		goals_sub = n.subscribe("goals", 2, &RosNode::goalsCallback, this);
		path_pub = n.advertise<nav_msgs::Path>("local_path", 2);
		gear_state_pub = n.advertise<std_msgs::UInt32>("gear_state",2);
		parking_complished_pub = n.advertise<std_msgs::UInt32>("parking_complished",2);

		task = light = motion = parking_space = -1;
//////////////////////////		
		isTrackDriving = false;
//////////////////////////
		//n.getParam("/isTrackDriving", isTrackDriving);
		//n.getParam("/stepsize_pp", stepsize_pp_value);
	}

	void missionstateCallback(const std_msgs::UInt32 & msg){
		int mask = 0b1111;
		int data = msg.data;
		motion = data & mask;
		light = (data>>4) & mask;
		task = (data>>8) & mask;
		cout << "motion : " << motion << " light : " << light << " task : " << task << endl;
	}

	void parkingspaceCallback(const std_msgs::Int32 & msg){
		parking_space = msg.data;
	}

	void goalsCallback(const nav_msgs::Path & msg){
		goals = msg;
	}

	void costmapCallback(const nav_msgs::OccupancyGrid & map){
		static int time_parking_complished{0};
		static int time_unparking_complished{0};
		static int gear_state{0};

/*
		isTrackDriving = false;
		task = OBSTACLE_STATIC;
		light = 0;
		motion = FORWARD_MOTION;
*/

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
			///////////////////
			//task = light = motion = 0;

			//if(goals.poses.empty()) return;
			//if(task == -1) return;
			
			int iternum;
			double radius;
			double stepsize;
			double threshold;
			double threshold2;
			double threshold2_obstacle_static;
			double cost_scale;
			double cost_scale_obstacle_static;
			n.getParam("/iternum", iternum);
			n.getParam("/radius", radius);
			n.getParam("/stepsize_rrt", stepsize);
			n.getParam("/threshold", threshold);
			n.getParam("/threshold2", threshold2);
			n.getParam("/threshold2_obstacle_static", threshold2_obstacle_static);
			n.getParam("/cost_scale", cost_scale); // 66-> 100 to 66
			n.getParam("/cost_scale_obstacle_static", cost_scale_obstacle_static); // 66-> 100 to 66


			if (task == OBSTACLE_STATIC){
				threshold2 = threshold2_obstacle_static;
				cost_scale = cost_scale_obstacle_static;
			}

			RRT rrt = RRT(iternum, radius, stepsize, threshold, threshold2);
			
			int t = clock();

			// get costmap	
			vector<vector<double>> cost_map(map.info.height,vector<double>(map.info.width));
			int h = map.info.height;
			int w = map.info.width;
			for(int i = 0; i<h; i++){
				for(int j = 0; j<w;j++){
					/*
					cost_map[i][j] = (double)(map.data[i*w+j]*cost_scale/100.0 + (101-cost_scale));
					*/

					if (task != OBSTACLE_STATIC)
						cost_map[i][j] = (double)(map.data[i*w+j]*cost_scale/100.0 + (101-cost_scale));
					else
						cost_map[i][j] = (double)(map.data[i*w+j]*cost_scale_obstacle_static/100.0 + (101-cost_scale_obstacle_static));
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
			Cor y = decision(goals.poses, cost_map, task, light, motion, parking_space, parking_complished_changed, unparking_complished_changed, gear_state);
			printf("goal : %lf %lf\n",y.x,y.y);
					
			if(parking_complished_changed){
				//time_parking_complished = (int)clock();
				time_parking_complished = (int)ros::Time::now().sec;
				// if  accidently clock() == 0
				if(time_parking_complished == 0) ++time_parking_complished;
			}
			//////////////////////////////////////////////////////////////////////
			// already stop but available path occur then stop  (because of localization error)
			else if(time_parking_complished != 0){
				y.x = 0; y.y = 0;
				duration_parking = ((int)ros::Time::now().sec - time_parking_complished);
				cout << "time : " << duration_parking << endl;
				if( duration_parking > 4 && duration_parking <=5){
					gear_state = 1;
				}
				else if (duration_parking > 5){
					gear_state = 1;
					time_parking_complished = 0;
				}
			}
			else if(unparking_complished_changed){
				gear_state = 0;
				std_msgs::UInt32 msg;
				parking_complished_pub.publish(msg);
				
				//time_unparking_complished = (int)clock();
				time_unparking_complished = (int)ros::Time::now().sec;
				// if  accidently clock() == 0
				if(time_unparking_complished == 0) ++time_unparking_complished;
			}
			else if(time_unparking_complished != 0){
				y.x = 0; y.y = 0;
				duration_unparking = ((int)ros::Time::now().sec - time_unparking_complished);
				cout << "time : " << duration_unparking << endl;
				if(duration_unparking > 1){
					gear_state = 0;
					time_unparking_complished = 0;
				}
			}
			cout << "parking_complished_changed : " << parking_complished_changed << endl;
			cout << "unparking_complished_changed : " << unparking_complished_changed << endl;
			cout << "gear_state : " << gear_state << endl;

			// parking...  pub to slam team
			std_msgs::UInt32 msg;
			msg.data = gear_state;
			gear_state_pub.publish(msg);
			
			/*
			if (duration_parking > 20){
					ros::Rate sleep_time{1};
					sleep_time.sleep();
					duration_parking = 0;
			}
			*/

			// stop!!!!!
			if(y.x == 0 && y.y == 0){
				nav_msgs::Path local_path;	
				path_pub.publish(local_path);
				cout << "stop at present location!\n";
				return;
			}

			y.y+=w/2;
			
			rrt.solve(path,cost_map,x, y, task == OBSTACLE_SUDDEN);

			for(Cor point :  path)
				cout << point.x << "," << point.y << endl;
			if(path.empty()){
				nav_msgs::Path local_path;	
				path_pub.publish(local_path);
				cout << "empty path is generated!\n";
				cout << "(obstacle located between goal and start point)\n";
				return;
			}

			// not sure
			cv::namedWindow("costmap_path");
			cv::Mat image(h,w,CV_8UC3);
			for(int i = 0;i<h;i++) for(int j = 0;j<w;j++){
					image.at<cv::Vec3b>(h-1-i,w-1-j)[0] = (cost_map[i][j]-(100-cost_scale))*100.0/(double)cost_scale;	
					image.at<cv::Vec3b>(h-1-i,w-1-j)[1] = (cost_map[i][j]-(100-cost_scale))*100.0/(double)cost_scale;	
					image.at<cv::Vec3b>(h-1-i,w-1-j)[2] = (cost_map[i][j]-(100-cost_scale))*100.0/(double)cost_scale;	
			}
/*
			if (task != OBSTACLE_STATIC){
				for(int i = 0;i<h;i++) for(int j = 0;j<w;j++){
					image.at<cv::Vec3b>(h-1-i,w-1-j)[0] = (cost_map[i][j]-(100-cost_scale))*100.0/(double)cost_scale;	
					image.at<cv::Vec3b>(h-1-i,w-1-j)[1] = (cost_map[i][j]-(100-cost_scale))*100.0/(double)cost_scale;	
					image.at<cv::Vec3b>(h-1-i,w-1-j)[2] = (cost_map[i][j]-(100-cost_scale))*100.0/(double)cost_scale;	
				}
			}
			else{
				for(int i = 0;i<h;i++) for(int j = 0;j<w;j++){
					image.at<cv::Vec3b>(h-1-i,w-1-j)[0] = (cost_map[i][j]-(100-cost_scale_obstacle_static))*100.0/(double)cost_scale_obstacle_static;	
					image.at<cv::Vec3b>(h-1-i,w-1-j)[1] = (cost_map[i][j]-(100-cost_scale_obstacle_static))*100.0/(double)cost_scale_obstacle_static;	
					image.at<cv::Vec3b>(h-1-i,w-1-j)[2] = (cost_map[i][j]-(100-cost_scale_obstacle_static))*100.0/(double)cost_scale_obstacle_static;	
				}
			}

*/
			for(int i = 0;i<path.size()-1;i++)
				line(image, cv::Point(w-1-path[i].y,h-1-path[i].x), cv::Point(w-1-path[i+1].y,h-1- path[i+1].x), cv::Scalar(100,200,50),1,0);

			cv::imshow("costmap_path",image);
			cv::waitKey(1);

			// convert path
			nav_msgs::Path local_path;


			// tracker use another coordinate system, heading = y axis!, right = x axis!
			int cnt = 0;
			double stepsz = 4.0;
			Cor cor;
			geometry_msgs::PoseStamped poseStamped;
			for(int i = 0;i<path.size();i++){
				cor = path[i];
				if(i==path.size()-1){
					poseStamped.header.seq = ++cnt;
					poseStamped.pose.position.x = - (cor.y-w/2);
					poseStamped.pose.position.y = cor.x;
					local_path.poses.push_back(poseStamped);
					continue;
				}
				Cor cor_next = path[i+1];
				double ds = sqrt((cor.x-cor_next.x)*(cor.x-cor_next.x) + (cor.y-cor_next.y)*(cor.y-cor_next.y));
				double dx = (cor_next.x - cor.x) * stepsz / ds;
				double dy = (cor_next.y - cor.y) * stepsz / ds;
				int stepcnt = 0;
				while(stepsz*stepcnt<ds){
					poseStamped.header.seq = ++cnt;
					poseStamped.pose.position.x = - (cor.y-w/2);
					poseStamped.pose.position.y = cor.x;
					local_path.poses.push_back(poseStamped);
					stepcnt++;
					cor.x+=dx;
					cor.y+=dy;
				}
			}

			poseStamped.header.seq = 0;
			local_path.poses.push_back(poseStamped);

			local_path.header.stamp.sec = 0;
			if (motion == HALT_MOTION)
				local_path.header.stamp.sec += 1;
			if (gear_state == 1)
				local_path.header.stamp.sec +=2;

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
