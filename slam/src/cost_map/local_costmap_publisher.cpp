#include <cmath>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/UInt32.h"

#include "slam/Data.h"

#include "ros/package.h"
#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "XYToPixel.h"


/*
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
*/

//std::stringstream ss;
//ss << ros::package::getPath("slam") << "src/mapping/costmap.png";

class Local_costmap_publisher{
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber costmap_sub;
		ros::Subscriber gear_state_sub;
		ros::Subscriber mission_state_sub;
		ros::Subscriber obstacle_cost_map_sub;
		ros::Publisher cost_map_pub;

		bool is_kcity;

	public:
		int map_size = 300;
		std::stringstream path_stream;
		cv::Mat glob_costmap;
		cv::Mat obstacle_costmap;
		int gear_state{0};
		int mission_state{0};
			
		//Constructor for local_path_publisher
		Local_costmap_publisher() {
			ros::param::get("/is_kcity", is_kcity);

			if(!is_kcity) path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_costmap.png";
			else path_stream << ros::package::getPath("slam") << "/config/KCity/KCity_costmap.png";
			glob_costmap = cv::imread(path_stream.str(), cv::IMREAD_GRAYSCALE);
			obstacle_costmap = cv::Mat(map_size,map_size, CV_8UC1, cv::Scalar(0));
			
			ROS_INFO("Image loaded");

			pub = nh.advertise<sensor_msgs::Image>("/local_costmap", 2);
			//subscribe for not considering rear driving
			//sub = nh.subscribe("/filtered_data", 2, &Local_costmap_publisher::callback, this);
			costmap_sub = nh.subscribe("/filtered_data", 2, &Local_costmap_publisher::callback, this);
			gear_state_sub = nh.subscribe("/gear_state", 2, &Local_costmap_publisher::gs_callback, this);
			mission_state_sub = nh.subscribe("/mission_state", 2, &Local_costmap_publisher::ms_callback, this);
			obstacle_cost_map_sub = nh.subscribe("/obstacle_map/costmap", 1, &Local_costmap_publisher::obstacle_cost_map_callback, this);
			cost_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/cost_map_with_goal_vector", 2);
		}
			
		int max(int a, int b){ return (a>b)?a:b;}

			//cv::Mat local_path_img = cv::Mat(300,300, CV_8UC3, cv::Scalar(255,255,255));
			//set path for own global costmap
			//cv::Mat glob_costmap = cv::imread("/home/parallels/catkin_ws/src/zero/slam/src/mapping/costmap.png", cv::IMREAD_GRAYSCALE);
			//cv::Mat local_costmap = cv::Mat(map_size,map_size, CV_8UC1, cv::Scalar(0));
			//const int channels = local_costmap.channels();
		void gs_callback(const std_msgs::UInt32 state){
			gear_state = state.data;
			std::cout << gear_state << std::endl;
		}

		void ms_callback(const std_msgs::UInt32 state){
			mission_state = state.data;
			std::cout << "mission state : " << mission_state << std::endl;
		}

		void obstacle_cost_map_callback(const sensor_msgs::Image::ConstPtr& msg){
			cv_bridge::CvImagePtr map_ptr;
			map_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
			obstacle_costmap = map_ptr->image;
		}

		void callback(const slam::Data data){
			cv::Mat local_costmap = cv::Mat(map_size,map_size, CV_8UC1, cv::Scalar(0));
			geometry_msgs::PoseStamped loc_pose;
			int curr_pixel_x{}, curr_pixel_y{};
			double step = 0.5;
			double pix_heading{};
						
			if(data.theta >= 0) pix_heading = data.theta;
			else pix_heading = data.theta + 2*M_PI;

			double head_coor_x, head_coor_y;
			head_coor_x = (step)*sin(pix_heading);
			head_coor_y = (step)*cos(pix_heading);
			XYToPixel(curr_pixel_x, curr_pixel_y, data.x, data.y, is_kcity);
						
			double point_pixel_x{}, point_pixel_y{};

			//forward driving
			if(!gear_state){
				for(int j=1; j<600; j++){
					point_pixel_x = curr_pixel_x + j*head_coor_y;
					point_pixel_y = curr_pixel_y - j*head_coor_x;
					for(int i=1; i<300; i++){
						point_pixel_x += head_coor_x;
						point_pixel_y += head_coor_y;
							
						local_costmap.at<uchar>(int(300-j*step),int(150+i*step)) = int(glob_costmap.at<uchar>(int(point_pixel_y), int(point_pixel_x)));
					}
				}

				for(int j=1; j<600; j++){
					point_pixel_x = curr_pixel_x + j*head_coor_y;
					point_pixel_y = curr_pixel_y - j*head_coor_x;
					for(int i=1; i<300; i++){
						point_pixel_x += -head_coor_x;
						point_pixel_y += -head_coor_y;

						local_costmap.at<uchar>(int(300-j*step),int(150-i*step)) = int(glob_costmap.at<uchar>(int(point_pixel_y), int(point_pixel_x)));
					}
				}

			}

			//rear driving
			else{
				std::cout << "rear driving" << std::endl;
				for(int j=201; j<800; j++){
					point_pixel_x = curr_pixel_x - j*head_coor_y;
					point_pixel_y = curr_pixel_y + j*head_coor_x;
					for(int i=1; i<300; i++){
						point_pixel_x -= head_coor_x;
						point_pixel_y -= head_coor_y;

						local_costmap.at<uchar>(int(300-(j-200)*step),int(150+i*step)) = int(glob_costmap.at<uchar>(int(point_pixel_y), int(point_pixel_x)));
					}
				}
				for(int j=201; j<800; j++){
					point_pixel_x = curr_pixel_x - j*head_coor_y;
					point_pixel_y = curr_pixel_y + j*head_coor_x;
					for(int i=1; i<300; i++){
						point_pixel_x += head_coor_x;
						point_pixel_y += head_coor_y;

						local_costmap.at<uchar>(int(300-(j-200)*step),int(150-i*step)) = int(glob_costmap.at<uchar>(int(point_pixel_y), int(point_pixel_x)));
					}
				}
			}
			
			// in the obstacle mission area
			if(mission_state == 2049){
				ROS_INFO("mission state is obstacle");
				for(int i = 0;i<300;i++){
					for(int j = 0;j<300;j++) local_costmap.at<uchar>(i,j) = max(local_costmap.at<uchar>(i,j), obstacle_costmap.at<uchar>(i,j));
				}
			}

			ROS_INFO("Publish cost map");

			nav_msgs::OccupancyGrid cost_map;
			cost_map.info.width = 300;
			cost_map.info.height = 300;

			for (int i = 1; i < 301; i++){
				for (int j = 1; j < 301; j++) cost_map.data.push_back(static_cast<int8_t>(local_costmap.at<uchar>(300-i,300-j)));
			}
						
			cost_map_pub.publish(cost_map);
			
			cv_bridge::CvImage img_bridge;
			sensor_msgs::Image img_msg;
			std_msgs::Header header;
			img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, local_costmap);
			img_bridge.toImageMsg(img_msg);
			pub.publish(img_msg);
		}
};

int main(int argc, char **argv){
	ros::init(argc, argv, "local_costmap_publisher");
	Local_costmap_publisher local_costmap_publisher;
	ros::spin();
}


