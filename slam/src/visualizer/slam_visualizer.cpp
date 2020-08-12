#include <iostream>
#include <string>
#include <sstream>
#include "ros/package.h"
#include <ros/ros.h>
#include <vector>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "XYToPixel.h"
#include "slam/Data.h"
#include <cmath>

class Slam_visualizer{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;

    public:

        int map_size = 300;
        std::stringstream path_stream;
        cv::Mat glob_map;

        Slam_visualizer(){
            path_stream << ros::package::getPath("slam") << "/config/global_path_visual.png";
            glob_map = cv::imread(path_stream.str(), cv::IMREAD_COLOR);

            ROS_INFO("Image Loaded");
            cv::namedWindow("local_map");
			sub = nh.subscribe("/filtered_data", 2, &Slam_visualizer::callback, this);
        }

        void callback(const slam::Data data){
            cv::Mat local_map = cv::Mat::zeros(map_size, map_size, CV_8UC3);
            int map_size{300};
            int curr_xpix{}, curr_ypix{};

            XYToPixel(curr_xpix, curr_ypix, data.x, data.y);

            cv::Point2f rc(curr_xpix, curr_ypix);
            cv::Mat matrix = cv::getRotationMatrix2D(rc, (M_PI_2-data.theta)*180/M_PI, 1);
            cv::Mat rotated_map(cv::Size(15000,15000), CV_8UC3);
			cv::warpAffine(glob_map, rotated_map, matrix, cv::Size(15000, 15000));

            int x = int(curr_xpix - map_size/2);
            int y = int(curr_ypix - map_size);

            local_map = rotated_map(cv::Rect(x, y, map_size, map_size));

            cv::imshow("local_map", local_map);
            cv::waitKey(0);
        }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "Slam_visualizer");
    Slam_visualizer slam_visualizer;
    ros::spin();
}
