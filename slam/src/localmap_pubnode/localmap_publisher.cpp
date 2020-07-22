#include <iostream>
#include "ros/ros.h"
#include <slam/Data.h>
#include "../mapping/map_cutter.h"
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

class Localmap_Publisher{
	private:
		ros::NodeHandle nh;
		ros::Publisher publisher;
		ros::Subscriber subscriber;

	public:
		Localmap_Publisher(){
			publisher = nh.advertise<sensor_msgs::Image>("/cutted_map", 1000);
			subscriber = nh.subscribe("/filtered_data", 1000, &Localmap_Publisher::callback, this);
		}

		void callback(const slam::Data data){
			cv::Mat gmap = cv::imread("map.png");
			cut_map(gmap, data.x, data.y, data.theta);
			cv_bridge::CvImage img_bridge;
			sensor_msgs::Image img_msg;
			std_msgs::Header header;
			img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, gmap);
			img_bridge.toImageMsg(img_msg);
			publisher.publish(img_msg);
		}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "Localmap_Publisher");
	Localmap_Publisher locmap_pub;
	ros::spin();
}