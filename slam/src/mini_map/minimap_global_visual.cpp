#include <iostream>
#include <ros/ros.h>
#include "ros/package.h"
#include "opencv2/opencv.hpp"
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include "XYToPixel.h"
#include <slam/Pixel.h>
#include <slam/Data.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

class MinimapGlobalVisual{
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;
		ros::Subscriber local_goals_sub;
		std::stringstream path_stream;

	public:
		cv::Mat global_map;
		cv::Mat mini_map = cv::Mat(1000,1000, CV_8UC3, cv::Scalar(0,0,0));

		MinimapGlobalVisual(){
			path_stream << ros::package::getPath("slam") << "/src/global_path/global_path.png";
			global_map = cv::imread(path_stream.str());

			pub = nh.advertise<sensor_msgs::Image>("/mini_map", 2);
			sub = nh.subscribe("/filtered_data", 2, &MinimapGlobalVisual::callback, this);
		}

		int prev_pixel_x{}, prev_pixel_y{};
		int count{0}, check{0};

		void callback(const slam::Data data){
			int inst_pixel_x, inst_pixel_y;
			int copy_pixel_x{}, copy_pixel_y{};

			double theta;
			XYToPixel(inst_pixel_x,inst_pixel_y,data.x,data.y);
			std::cout<<inst_pixel_x << inst_pixel_y << std::endl;
			bool x_500{inst_pixel_x <= 500}, y_500{inst_pixel_y <= 500}, x_14500{inst_pixel_x >= 14500}, y_14500{inst_pixel_y >= 14500};
			std::cout << x_500 << "," << y_500 << "," << x_14500 << "," << y_14500 << std::endl;

			if(count == 0){
				prev_pixel_x = inst_pixel_x;
				prev_pixel_y = inst_pixel_y;
				check = 1;
			}

			else if(((inst_pixel_x-prev_pixel_x)*(inst_pixel_x-prev_pixel_x)+(inst_pixel_y-prev_pixel_y)*(inst_pixel_y-prev_pixel_y))>80){
				prev_pixel_x = inst_pixel_x;
				prev_pixel_y = inst_pixel_y;
				check = 1;
			}

			if(check==1){
				cv::circle(global_map, cv::Point(inst_pixel_x, inst_pixel_y), 2, cv::Scalar(255,0,0), -1);
				count++;
			}

			if(!x_500 && !y_500 && !x_14500 && !y_14500){
				std::cout << "on map" << std::endl;
				for(int i = 0; i < 1000; i++){
					copy_pixel_y = inst_pixel_y - 500 + i;
					for (int j = 0; j < 1000; j ++){
						copy_pixel_x = inst_pixel_x - 500 + j;
						mini_map.at<cv::Vec3b>(i, j)[0] = global_map.at<cv::Vec3b>(copy_pixel_y, copy_pixel_x)[0];
						mini_map.at<cv::Vec3b>(i, j)[1] = global_map.at<cv::Vec3b>(copy_pixel_y, copy_pixel_x)[1];
						mini_map.at<cv::Vec3b>(i, j)[2] = global_map.at<cv::Vec3b>(copy_pixel_y, copy_pixel_x)[2];

					}
				}
			}
			
			cv_bridge::CvImage img_bridge;
			sensor_msgs::Image img_msg;
			std_msgs::Header header;
			img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, mini_map);
			printf("image converting\n");
			img_bridge.toImageMsg(img_msg);
			printf("image converted!\n");
			pub.publish(img_msg);

		}
	
};

int main(int argc, char **argv){
	ros::init(argc, argv, "minimap_global_visual");
	MinimapGlobalVisual minimapglobalvisual;
	ros::spin();
}
