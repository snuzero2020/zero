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
		ros::Subscriber sub;

		message_filters::Subscriber<sensor_msgs::Image> local_costmap;
                message_filters::Subscriber<nav_msgs::Path> goals;
                typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Path> sync_policy;
		typedef message_filters::Synchronizer<sync_policy> Sync;
		boost::shared_ptr<Sync> sync_;

	public:
		cv::Mat visualized_img = cv::Mat::zeros(300, 300, CV_8UC3);

		pp_visualizer(){
			local_costmap.subscribe(nh, "local_costmap", 10);
			goals.subscribe(nh, "goals", 10);
			sync_.reset(new Sync(sync_policy(10), local_costmap, goals));
			cv::namedWindow("pp_visualizer");
			sync_->registerCallback(boost::bind(&pp_visualizer::callback, this, _1, _2));
		}
		
		void callback(const sensor_msgs::ImageConstPtr local_costmap, const nav_msgs::PathConstPtr globpath_nearby){
			cv_bridge::CvImagePtr cv_ptr;
			cv_ptr = cv_bridge::toCvCopy(local_costmap, "bgr8");
			//drawing the local_costmap on visualized_img
			visualized_img = cv_ptr->image;

			//visualizing the paths from globpath_nearby into the image
			for(int i=0; i < globpath_nearby->poses.size(); i++){
				cv::circle(visualized_img, cv::Point(300-globpath_nearby->poses[i].pose.position.y, 150+globpath_nearby->poses[i].pose.position.x), 2, cv::Scalar(255,255,255),-1);
				//cv::arrowedLine(visualized_img, cv::Point(150+globpath_nearby->poses[i].pose.position.x, 300-globpath_nearby->poses[i].pose.position.y), cv::Point(150+(globpath_nearby->poses[i].pose.position.x) + 5*cos(globpath_nearby->poses[i].pose.position.z*2*M_PI/180), 300 - (globpath_nearby->poses[i].pose.position.y) - 5*sin(globpath_nearby->poses[i].pose.position.z*2*M_PI/180)), 2, 4, 1, 2);
			}
			//cv::ellipse(visualized_img, cv::Point(150,300), cv::Size(80,150), 0, M_PI, (0,255,0), 3);
			cv::imshow("pp_visualizer", visualized_img);
			cv::waitKey(1);
		}
};

int main(int argc, char**argv){
	ros::init(argc, argv, "pp_visualizer");
	pp_visualizer pp_visualizer;
	ros::spin();
}
