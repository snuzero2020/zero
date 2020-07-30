#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <cv_bridge/cv_bridge.h>
#include <slam/Data.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>


class pp_visualizer{
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;

		message_filters::Subscriber<sensor_msgs::Image> local_costmap;
                message_filters::Subscriber<nav_msgs::Path> globpath_nearby;
                typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Path> sync_policy;
		typedef message_filters::Synchronizer<sync_policy> Sync;
		boost::shared_ptr<Sync> sync_;
	
	public:
		pp_visualizer(){
			local_costmap.subscribe(nh, "local_costmap", 10);
			globpath_nearby.subscribe(nh, "globpath_nearby", 10);
			sync_.reset(new Sync(sync_policy(10), local_costmap, globpath_nearby));
			sync_->registerCallback(boost::bind(&pp_visualizer::callback, this, _1, _2));
			pub = nh.advertise<sensor_msgs::Image>("visualized_img", 100);
		}
		
		void callback(const sensor_msgs::ImageConstPtr local_costmap, const nav_msgs::PathConstPtr globpath_nearby){
			//sensor_msgs::CvBridge bridge_;
			//IplImage* image;
			cv_bridge::CvImagePtr cv_ptr;
			//image = bridge_.imgMsgToCv(local_costmap, "bgr8");
			cv_ptr = cv_bridge::toCvCopy(local_costmap, "bgr8");
			//drawing the local_costmap on visualized_img
			cv::Mat visualized_img = cv_ptr->image;

			//visualizing the paths from globpath_nearby into the image
			for(int i=0; i < globpath_nearby->poses.size(); i++){
			/*
				visualized_img.at<cv::Vec3b>(globpath_nearby.poses[i].x, globpath_nearby.poses[i].y)[0] = 255;
				visualized_img.at<cv::Vec3b>(globpath_nearby.poses[i].x, globpath_nearby.poses[i].y)[1] = 255;
				visualized_img.at<cv::Vec3b>(globpath_nearby.poses[i].x, globpath_nearby.poses[i].y)[2] = 255;
			*/
				cv::circle(visualized_img, cv::Point(150+globpath_nearby->poses[i].pose.position.x, 300-globpath_nearby->poses[i].pose.position.y), 3, cv::Scalar(255,255,255),-1);
			//	cv::arrowedLine(visualized_img, cv::Point(150+globpath_nearby->poses[i].pose.position.x, 300-globpath_nearby->poses[i].pose.position.y), cv::Point(150+(globpath_nearby->poses[i].pose.position.x) + 5*cos(globpath_nearby->poses[i].pose.position.z*2*M_PI/180), 300 - (globpath_nearby->poses[i].pose.position.y) - 5*sin(globpath_nearby->poses[i].pose.position.z*2*M_PI/180)), 2, 4, 1, 2);
			
			}
			//cv::ellipse(visualized_img, cv::Point(150,300), cv::Size(80,150), 0, M_PI, (0,255,0), 3);
			cv_bridge::CvImage img_bridge;
			sensor_msgs::Image img_msg;
			std_msgs::Header header;
			img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, visualized_img);
			img_bridge.toImageMsg(img_msg);
			pub.publish(img_msg);
		}
};

int main(int argc, char**argv){
	ros::init(argc, argv, "pp_visualizer");
	pp_visualizer pp_visualizer;
	ros::spin();
}
