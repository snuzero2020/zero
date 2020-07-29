#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include "opencv2/opencv2.hpp"
#include <cv_bridge/cv_bridge.h>
#include <slam/Data.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>


class pp_visualizer{
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;

	public:
		pp_visualizer(){

			message_filters::Subscriber<sensor_msgs::Image> local_costmap(nh, "name of local cost map topic ", 10);
			message_filters::Subscriber<nav_msgs::Path> globpath_nearby(nh, "globpath_nearby",10);
			TimeSynchronizer<sensor_msgs::Image, nav_msgs::Path> sync(local_costmap, globpath_nearby, 10);
			sync.registerCallback(boost::bind(&pp_visualizer::callback, _1, _2);
			pub = nh.advertise<sensor_msgs::Image>("visualized_img", 100);
		}
		void callback(const sensor_msgs::Image local_costmap, const nav_msgs::Path globpath_nearby){
			//drawing the local_costmap on visualized_img
			cv::Mat visualized_img = cv::imread(local_costmap);
			//visualizing the paths from globpath_nearby into the image
			for(int i=0; i < globpath_nearby.poses.size(); i++){
			/*
				visualized_img.at<cv::Vec3b>(globpath_nearby.poses[i].x, globpath_nearby.poses[i].y)[0] = 255;
				visualized_img.at<cv::Vec3b>(globpath_nearby.poses[i].x, globpath_nearby.poses[i].y)[1] = 255;
				visualized_img.at<cv::Vec3b>(globpath_nearby.poses[i].x, globpath_nearby.poses[i].y)[2] = 255;
			*/
				cv::circle(visualized_img, cv::Point(globpath_nearby.poses[i].position.x, globpath_nearby.poses[i].position.y), 2, cv::Scalar(255,255,255),-1);
				cv::arrowedLine(visualized_img, cv::Point(globpath_nearby.poses[i].position.x, globpath_nearby.poses[i].position.y), cv::Point(globpath_nearby.poses[i].position.x + 5*cos(globpath_nearby.poses[i].position.z*2*M_PI/180), globpath_nearby.poses[i].position.y- 5*sin(globpath_nearby.poses[i].position.z*2*M_PI/180)), 2, 4, 1, 2)
				//need to add arrows
			}
			cv::ellipse(visualized_img, cv::Point(150,300), cv::Size(80,150), 0, M_PI, (0,255,0), 3);
			cv_bridge::CvImage img_bridge;
			sensor_msgs::Image img_msg;
			std_msgs::Header header;
			img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, visualized_img));
			img_bridge.toImageMsg(img_msg);
			pub.publish(img_msg);
		}
};

int main(int arc, char**argv){
	ros::init(argc, argv, "pp_visualizer");
	pp_visualizer pp_visualizer;
	ros::spin()
}
