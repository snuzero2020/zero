#include <iostream>
#include <string>
#include <sstream>
#include "ros/package.h"
#include <ros/ros.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include "XYToPixel.h"
#include "slam/Data.h"
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <cmath>

//std::stringstream ss;
//ss << ros::package::getPath("slam") << "src/mapping/costmap.png";

class Local_costmap_publisher{
        private:
                ros::NodeHandle nh;
                ros::Publisher pub;
                ros::Subscriber sub;

        public:

		int map_size = 300;

                //Constructor for local_path_publisher
                Local_costmap_publisher(){
                        pub = nh.advertise<sensor_msgs::Image>("/local_costmap", 10);
                        sub = nh.subscribe("/filtered_data", 1000, &Local_costmap_publisher::callback, this);
                }


                //cv::Mat local_path_img = cv::Mat(300,300, CV_8UC3, cv::Scalar(255,255,255));
		//set path for own global costmap
		cv::Mat glob_costmap = cv::imread("/home/parallels/catkin_ws/src/zero/slam/src/mapping/costmap.png", cv::IMREAD_GRAYSCALE);
		//cv::Mat local_costmap = cv::Mat(map_size,map_size, CV_8UC1, cv::Scalar(0));
		//const int channels = local_costmap.channels();

                void callback(const slam::Data data){
			cv::Mat local_costmap = cv::Mat(map_size,map_size, CV_8UC1, cv::Scalar(0));
			geometry_msgs::PoseStamped loc_pose;
                        int curr_pixel_x{}, curr_pixel_y{};
                        double step = 0.5;
			double pix_heading{};
                        
			if(data.theta >= 0) pix_heading = data.theta;
                        else pix_heading = data.theta + M_2_PI;

			double head_coor_x, head_coor_y;
                        head_coor_x = (step)*sin(pix_heading);
                        head_coor_y = (step)*cos(pix_heading);
			XYToPixel(glob_costmap, data.x, data.y, curr_pixel_x, curr_pixel_y, 2);
                        
			double point_pixel_x{}, point_pixel_y{};

                        for(int j=1; j<600; j++){
                                point_pixel_x = curr_pixel_x - j*head_coor_y;
                                point_pixel_y = curr_pixel_y + j*head_coor_x;
                                for(int i=1; i<300; i++){
                                        point_pixel_x += head_coor_x;
                                        point_pixel_y += head_coor_y;
							
					local_costmap.at<uchar>(int(150+i*step),int(300-j*step)) = int(glob_costmap.at<uchar>(int(point_pixel_x), int(point_pixel_y)));
				}
                        }
                        for(int j=1; j<600; j++){
                                point_pixel_x = curr_pixel_x - j*head_coor_y;
                                point_pixel_y = curr_pixel_y + j*head_coor_x;
                                for(int i=1; i<300; i++){
                                        point_pixel_x += -head_coor_x;
                                        point_pixel_y += -head_coor_y;

					local_costmap.at<uchar>(int(150-i*step),int(300-j*step)) = int(glob_costmap.at<uchar>(int(point_pixel_x), int(point_pixel_y)));
				
                                }
                        }
			
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

