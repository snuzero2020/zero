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

class map_tracer{
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;
		std::stringstream path_stream;

	public:
		//set the right path for your map
		cv::Mat glob_map;
		//cv::Mat glob_map = cv::imread("/home/junseolee/catkin_ws/src/zero/slam/src/mapping/map.png");
		cv::Mat mini_map = cv::Mat(1000,1000, CV_8UC3, cv::Scalar(0,0,0));
		map_tracer(){
			path_stream << ros::package::getPath("slam") << "/src/mapping/map.png";
			glob_map = cv::imread(path_stream.str(), cv::IMREAD_GRAYSCALE);
			ROS_INFO("Image loaded");
			pub = nh.advertise<sensor_msgs::Image>("/mini_map", 2);
			sub = nh.subscribe("/filtered_data", 2, &map_tracer::callback, this);
		}

		int prev_pixel_x{}, prev_pixel_y{};
		int count{0}, check{0};

		void callback(const slam::Data data){
			int inst_pixel_x, inst_pixel_y;
			int copy_pixel_x{}, copy_pixel_y{};
                        //inst_pixel_x = data.x;
		        //inst_pixel_y = data.y;
			XYToPixel(inst_pixel_x,inst_pixel_y,data.x,data.y);
			std::cout<<inst_pixel_x << inst_pixel_y << std::endl;
			bool x_500{inst_pixel_x <= 500}, y_500{inst_pixel_y <= 500}, x_14500{inst_pixel_x >= 14500}, y_14500{inst_pixel_y >= 14500};
			std::cout << x_500 << "," << y_500 << "," << x_14500 << "," << y_14500 << std::endl;	

			if(count == 0){
                                prev_pixel_x = inst_pixel_x;
                                prev_pixel_y = inst_pixel_y;
                          //      std::cout << "initial pose" << std::endl;
                                check = 1;
                        }
                        else if(((inst_pixel_x-prev_pixel_x)*(inst_pixel_x-prev_pixel_x)+(inst_pixel_y-prev_pixel_y)*(inst_pixel_y-prev_pixel_y))>80){
                                prev_pixel_x = inst_pixel_x;
                                prev_pixel_y = inst_pixel_y;
                            //    std::cout << "pixel fixed" << std::endl;
                                check = 1;
                        }
                        if(check==1){
				cv::circle(glob_map, cv::Point(inst_pixel_x, inst_pixel_y), 2, cv::Scalar(255,0,0), -1);
			//	std::cout << "pixel filled" << std::endl;
                                count++;
                        }
			
			if(!x_500 && !y_500 && !x_14500 && !y_14500){
				std::cout << "on map" << std::endl;
				for(int i=0; i<1000; i++){
					copy_pixel_y = inst_pixel_y - 500 + i;
					for(int j=0; j<1000; j++){
						copy_pixel_x = inst_pixel_x - 500 + j;
						mini_map.at<cv::Vec3b>(i, j)[0] = glob_map.at<cv::Vec3b>(copy_pixel_y, copy_pixel_x)[0];
						mini_map.at<cv::Vec3b>(i, j)[1] = glob_map.at<cv::Vec3b>(copy_pixel_y, copy_pixel_x)[1];
						mini_map.at<cv::Vec3b>(i, j)[2] = glob_map.at<cv::Vec3b>(copy_pixel_y, copy_pixel_x)[2];
					}
				}
				cv::circle(mini_map, cv::Point(500,500), 3, cv::Scalar(0,255,0), -1);
				cv_bridge::CvImage img_bridge;
                        	sensor_msgs::Image img_msg;
                        	std_msgs::Header header;
                        	img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, mini_map);
                        	printf("image converting\n");
                       		img_bridge.toImageMsg(img_msg);
                        	printf("image converted!\n");
                        	pub.publish(img_msg);
			}
		}
};

int main(int argc, char **argv){
	ros::init(argc, argv, "minimap_publisher");
	map_tracer minimap_publisher;
	ros::spin();
}
