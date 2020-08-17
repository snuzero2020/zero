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
		ros::Publisher cost_map_pub;

		bool is_kcity;

	public:

		int map_size = 300;
		std::stringstream path_stream;
		cv::Mat glob_costmap;
		int gear_state{0};
			
		//Constructor for local_path_publisher
		Local_costmap_publisher() {
			ros::param::get("/is_kcity", is_kcity);

			path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_costmap.png";
			glob_costmap = cv::imread(path_stream.str(), cv::IMREAD_GRAYSCALE);
			ROS_INFO("Image loaded");

			pub = nh.advertise<sensor_msgs::Image>("/local_costmap", 2);
			//subscribe for not considering rear driving
			//sub = nh.subscribe("/filtered_data", 2, &Local_costmap_publisher::callback, this);
			costmap_sub = nh.subscribe("/filtered_data", 2, &Local_costmap_publisher::callback, this);
			gear_state_sub = nh.subscribe("/gear_state", 2, &Local_costmap_publisher::gs_callback, this);
			cost_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("/cost_map_with_goal_vector", 2);
		}
			


			//cv::Mat local_path_img = cv::Mat(300,300, CV_8UC3, cv::Scalar(255,255,255));
			//set path for own global costmap
			//cv::Mat glob_costmap = cv::imread("/home/parallels/catkin_ws/src/zero/slam/src/mapping/costmap.png", cv::IMREAD_GRAYSCALE);
			//cv::Mat local_costmap = cv::Mat(map_size,map_size, CV_8UC1, cv::Scalar(0));
			//const int channels = local_costmap.channels();
		void gs_callback(const std_msgs::UInt32 state){
			gear_state = state.data;
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

				nav_msgs::OccupancyGrid cost_map;
				cost_map.info.width = 300;
				cost_map.info.height = 300;

				for (int i = 1; i < 301; i++){
					for (int j = 1; j < 301; j++) cost_map.data.push_back((int8_t)local_costmap.at<uchar>(300-i,300-j)); 
				}
			}

			//rear driving
			else{
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
			
			nav_msgs::OccupancyGrid cost_map;
			cost_map.info.width = 300;
			cost_map.info.height = 300;

			for (int i = 1; i < 301; i++){
				for (int j = 1; j < 301; j++) cost_map.data.push_back(static_cast<int8_t>local_costmap.at<uchar>(300-i,300-j));
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


