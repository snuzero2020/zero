#include <iostream>
#include <vector>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <map>
#include <string>
#include <sstream>

#include "slam/Pixel.h"
#include "slam/Data.h"
#include "slam/GlobalPathPoint.h"

#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/Path.h"
#include "std_msgs/UInt32.h"

#include "ros/ros.h"
#include "ros/package.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"

#include "XYToPixel.h"

using namespace std;

class map_tracer{
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber sub;
		std::stringstream path_stream1;
		std::stringstream path_stream2;
		bool is_kcity;

		vector<slam::GlobalPathPoint> global_path_;
		const char delimiter_ = ' ';

	public:
		//set the right path for your map
		cv::Mat glob_map;
		cv::Mat mini_map = cv::Mat(1000,1000, CV_8UC3, cv::Scalar(0,0,0));
		
		nav_msgs::Path local_path;
		
		map_tracer(){
			//ros::param::get("/is_kcity", is_kcity);

			is_kcity = false;
			if(is_kcity){
				path_stream1 << ros::package::getPath("slam") << "/config/KCity/KCity.png";
				path_stream2 << ros::package::getPath("slam") << "/config/KCity/re_global_path.txt";
			}
			else{
				path_stream1 << ros::package::getPath("slam") << "/config/FMTC/FMTC_map.png";
				path_stream2 << ros::package::getPath("slam") << "/config/FMTC/FMTC_global_path.txt";
			}
			glob_map = cv::imread(path_stream1.str());
			
			if(glob_map.empty()) ROS_INFO("no global map");
			else ROS_INFO("global map loaded");
			
			ROS_INFO("global map loaded");
			cv::namedWindow("minimap");
			pub = nh.advertise<sensor_msgs::Image>("/mini_map", 2);
			sub = nh.subscribe("/filtered_data", 100, &map_tracer::callback, this);
			load_global_path();
		}

		int prev_pixel_x{}, prev_pixel_y{};
		int count{0}, check{0};

		void callback(const slam::Data data){
			int inst_pixel_x, inst_pixel_y;
			int copy_pixel_x{}, copy_pixel_y{};
            //inst_pixel_x = data.x;
		    //inst_pixel_y = data.y;
			double theta;
			theta = data.theta;

			XYToPixel(inst_pixel_x, inst_pixel_y, data.x, data.y, is_kcity);	
			std::cout<<inst_pixel_x << inst_pixel_y << std::endl;
			bool x_500{inst_pixel_x <= 500}, y_500{inst_pixel_y <= 500}, x_11572{inst_pixel_x >= 11572}, y_13726{inst_pixel_y >= 13726};
			std::cout << x_500 << "," << y_500 << "," << x_11572 << "," << y_13726 << std::endl;	

			if(count == 0){
				prev_pixel_x = inst_pixel_x;
				prev_pixel_y = inst_pixel_y;
				// std::cout << "initial pose" << std::endl;
				check = 1;
			}
			else if(((inst_pixel_x-prev_pixel_x)*(inst_pixel_x-prev_pixel_x)+(inst_pixel_y-prev_pixel_y)*(inst_pixel_y-prev_pixel_y))>80){
				prev_pixel_x = inst_pixel_x;
				prev_pixel_y = inst_pixel_y;
				// std::cout << "pixel fixed" << std::endl;
				check = 1;
			}
			if(check==1){
				cv::circle(glob_map, cv::Point(inst_pixel_x, inst_pixel_y), 4, cv::Scalar(0,0,0), -1);
				// std::cout << "pixel filled" << std::endl;
				count++;
			}
			
			if(!x_500 && !y_500 && !x_11572 && !y_13726){
				std::cout << "on map" << std::endl;
				//mini_map = glob_map(cv::Rect(inst_pixel_x-500,inst_pixel_y-500,1000,1000)).clone();
				for(int i=0; i<1000; i++){
				  	copy_pixel_y = inst_pixel_y - 500 + i;
				  	for(int j=0; j<1000; j++){
				  		copy_pixel_x = inst_pixel_x - 500 + j;
				  		mini_map.at<cv::Vec3b>(i, j)[0] = glob_map.at<cv::Vec3b>(copy_pixel_y, copy_pixel_x)[0];
				  		mini_map.at<cv::Vec3b>(i, j)[1] = glob_map.at<cv::Vec3b>(copy_pixel_y, copy_pixel_x)[1];
				  		mini_map.at<cv::Vec3b>(i, j)[2] = glob_map.at<cv::Vec3b>(copy_pixel_y, copy_pixel_x)[2];
				 		
					}
				}
				//global_to_local(data);
				int cnt = 1;
                slam::GlobalPathPoint pose;
				geometry_msgs::PoseStamped pose_change;
				for (auto iter = global_path_.begin(); iter != global_path_.end(); iter++, cnt++){

                    pose.x = (*iter).x;
                    pose.y = (*iter).y;
                    pose.theta = (*iter).theta;
                    pose.flag = ((*iter).flag) | (cnt<<4);

                    double X = pose.x - data.x;
                    double Y = pose.y - data.y;

                    if(abs(X) < 14.9 || abs(Y) < 14.9){
                        pose_change.pose.position.x = int(X/(0.03));
                        pose_change.pose.position.y = int(Y/(0.03));
                        pose_change.pose.position.z = pose.theta;
                        pose_change.header.seq = pose.flag;

						switch(pose_change.header.seq){
				 			case 0:
								cv::circle(mini_map, cv::Point(pose_change.pose.position.x+500, 500-pose_change.pose.position.y), 4, cv::Scalar(250,0,0), -1);
				 				break;
				 			case 1:	
				 				cv::circle(mini_map, cv::Point(pose_change.pose.position.x+500, 500-pose_change.pose.position.y), 4, cv::Scalar(0,250,0), -1);
								break;
							case 2:
								cv::circle(mini_map, cv::Point(pose_change.pose.position.x+500, 500-pose_change.pose.position.y), 4, cv::Scalar(0,0,250), -1);
								break;
							case 3:
		 						cv::circle(mini_map, cv::Point(pose_change.pose.position.x+500, 500-pose_change.pose.position.y), 4, cv::Scalar(125,125,0), -1);
								break;
							case 4:
								cv::circle(mini_map, cv::Point(pose_change.pose.position.x+500, 500-pose_change.pose.position.y), 4, cv::Scalar(0,125,125), -1);
								break;
						}
					}
				}
				cv::circle(mini_map, cv::Point(500,500), 3, cv::Scalar(0,255,0), -1);
				cv::arrowedLine(mini_map, cv::Point(500,500), cv::Point(500+50*cos(theta), 500-50*sin(theta)), (0,0,255), 8, 0, 0.5);
				
				cv::imshow("minimap", mini_map);
				int key = cv::waitKey(30);
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

		void load_global_path(){ 
        	string in_line; 
        	ifstream in(path_stream2.str()); 
        	while(getline(in, in_line)){ 
        	    stringstream ss(in_line); 
        	    string token; 
        	    vector<string> result; 
        	    while(getline(ss, token, delimiter_)){ 
        	        result.push_back(token); 
        	    } 
        	    slam::GlobalPathPoint point; 
        	if (result.size() == 0) break; 
        	else if(result.at(0) == "//") continue; 
        	    point.x = stod(result.at(0)); 
        	    point.y = stod(result.at(1)); 
        	    point.theta = stod(result.at(2)); 
        	    point.flag = stod(result.at(3)); 
        	    cout << point.x << " " << point.y << " " << point.theta << " " << point.flag << endl;  
        	    global_path_.push_back(point); 
        	} 
    	}

		void global_to_local(const slam::Data msg){
    	    local_path.poses.clear();
    	    slam::GlobalPathPoint pose;
    	    geometry_msgs::PoseStamped pose_change;
    	    int cnt = 1;
    	        for (auto iter = global_path_.begin(); iter != global_path_.end(); iter++, cnt++){
	
	                pose.x = (*iter).x;
	                pose.y = (*iter).y;
	                pose.theta = (*iter).theta;
	                pose.flag = ((*iter).flag) | (cnt<<4);
	
	                double X = pose.x - msg.x;
	                double Y = pose.y - msg.y;
	            	
					if(abs(X) < 14.9 || abs(Y) < 14.9){
						pose_change.pose.position.x = int(X/(0.03));
						pose_change.pose.position.y = int(Y/(0.03));
						pose_change.pose.position.z = pose.theta;
						pose_change.header.seq = pose.flag;
	
						local_path.poses.push_back(pose_change);
					}
				}
			}
};

int main(int argc, char **argv){
	ros::init(argc, argv, "minimap_publisher");
	map_tracer minimap_publisher;
	ros::spin();
}
