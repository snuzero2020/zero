#include <iostream>
#include "ros/ros.h"
#include <slam/Data.h>
#include "../mapping/MapCutter.hpp"
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
		static int count;

		Localmap_Publisher(){
			publisher = nh.advertise<sensor_msgs::Image>("/cutted_map", 1000);
			subscriber = nh.subscribe("/filtered_data", 1000, &Localmap_Publisher::callback, this);
		}

		void callback(const slam::Data data){
			//cv::Mat gmap = cv::imread("/home/jeongwoooh/catkin_ws/src/zero/slam/src/mapping/map.png", cv::IMREAD_COLOR);

			MapCutter map_cutter(2);
			cv::Mat gmap = map_cutter.smartCut(data.x, data.y, data.theta);
			if ((count % 300) == 0) {
				cv::imwrite("src/zero/slam/src/mapping/cut_map.png", gmap);
			}
			
			printf("map cut&saved~!\n");
			cv_bridge::CvImage img_bridge;
			sensor_msgs::Image img_msg;
			std_msgs::Header header;
			img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, gmap);
			printf("image converting\n");
			img_bridge.toImageMsg(img_msg);
			printf("image converted!\n");
			//imshow("Result", gmap);
			//waitKey(0);
			publisher.publish(img_msg);

			count++;
			cout << count << endl;
		}
};

int Localmap_Publisher::count = 0;

int main(int argc, char** argv){
	MapCutter::loadMap();
	//Localmap_Publisher::count = 0;

	ros::init(argc, argv, "Localmap_Publisher");
	Localmap_Publisher Localmap_Publisher;
	ros::spin();
}
