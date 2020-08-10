#include <iostream>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include <vector>
#include "XYToPixel.h"
#include "slam/Data.h"
#include <nav_msgs/Path.h>
#include <cmath>
#include "ros/package.h"

class Global_path_gen{
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;
		std::stringstream path_stream;

	public:
	
		int prev_pixel_x{}; 
		int prev_pixel_y{};
		int count{0};
		nav_msgs::Path global_path;
		cv::Mat path_map;

		Global_path_gen(){
			sub = nh.subscribe("/filtered_data", 1000, &Global_path_gen::callback, this);
			path_stream << ros::package::getPath("slam") << "/config/glob_path.png";	
			path_map = cv::imread(path_stream.str());
			if(path_map.empty()){
				path_map = cv::Mat(15000, 15000, CV_8UC3, cv::Scalar(255,255,255));
				ROS_INFO("Image initiated");
			}else{
				ROS_INFO("Image loaded");
			}
		}
		
		void callback(const slam::Data data){
			geometry_msgs::PoseStamped inst_pose;
			int inst_pixel_x, inst_pixel_y;
			int  check{0};	
			//change the last parameter in XYToPixel to choose the specific map
			XYToPixel(inst_pixel_x, inst_pixel_y, data.x, data.y);
			std::cout<<inst_pixel_x<<" "<<inst_pixel_y<<std::endl;
			if(global_path.poses.empty()){
				inst_pose.pose.position.x = inst_pixel_x;
				inst_pose.pose.position.y = inst_pixel_y;
				inst_pose.pose.position.z = data.theta;
				prev_pixel_x = inst_pixel_x;
				prev_pixel_y = inst_pixel_y;
				std::cout << "initial pose" << std::endl;
				check = 1;
			}
			else if(((inst_pixel_x-prev_pixel_x)*(inst_pixel_x-prev_pixel_x)+(inst_pixel_y-prev_pixel_y)*(inst_pixel_y-prev_pixel_y))>98){
				inst_pose.pose.position.x = inst_pixel_x;
				inst_pose.pose.position.y = inst_pixel_y;
				inst_pose.pose.position.z = data.theta;
				prev_pixel_x = inst_pixel_x;
				prev_pixel_y = inst_pixel_y;
				std::cout << "pixel fixed" << std::endl;
				check = 1;
			}
			if(check==1){
				global_path.poses.push_back(inst_pose);
				
				if(data.theta >= 0){	
					path_map.at<cv::Vec3b>(inst_pixel_y, inst_pixel_x)[0] = int(data.theta*180/M_PI)/2;
				}
				else{
					path_map.at<cv::Vec3b>(inst_pixel_y, inst_pixel_x)[0] = int(data.theta*180/M_PI+360)/2;
				}
				
				path_map.at<cv::Vec3b>(inst_pixel_y, inst_pixel_x)[1] = 0;
				path_map.at<cv::Vec3b>(inst_pixel_y, inst_pixel_x)[2] = 0;

				std::cout << "pixel filled" << std::endl;
				//if(count%3==0) {
					//cv::imwrite("/home/junseolee/catkin_ws/src/zero/slam/src/global_path/glob_path.png", path_map);
					cv::imwrite(path_stream.str(), path_map);
					std::cout << "image saved" << std::endl;
				//}
				count++;
			}

		}
};

int main(int argc, char **argv){
	ros::init(argc, argv, "globpath_generator");
	Global_path_gen global_path_gen;
	ros::spin();
}
