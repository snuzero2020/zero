#include "ros/ros.h"
#include <iostream>
#include "opencv2/opencv.hpp"
#include <math.h>
#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include <cmath>
#include "slam/GlobalPathPoint.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include "ros/package.h"
#include <vector>
#include <cv_bridge/cv_bridge.h>
#include "XYToPixel.h"
#include <slam/Pixel.h>
#include <slam/Data.h>
#include "XYToPixel.h"

using namespace std;
using namespace cv;

class GlobalVisual{
    
    private:
        const char delimiter_ = ' '; 
        string input_global_path = "/home/lee/catkin_ws/src/zero/slam/src/global_path/global_path.txt";
        string input_global_map = "/home/lee/catkin_ws/src/zero/slam/src/mapping/costmap.png";
        ros::NodeHandle nh;
		ros::Publisher pub;

    public:
        cv::Mat img = imread(input_global_map, IMREAD_COLOR);

        GlobalVisual(){
            pub = nh.advertise<sensor_msgs::Image>("/global_map_with_path", 2);
            global_path_visual();
        }

        void global_path_visual(){
            if (img.empty()){
                cout << "fuck" << endl;
            }
            cout  << 1 << endl;
            string in_line;
            ifstream in(input_global_path);

            while(getline(in, in_line)){
                stringstream ss(in_line);
                string token;
                vector<string> result;
                while(getline(ss, token, delimiter_)){
                    result.push_back(token);
                }
                
                slam::GlobalPathPoint point;
                point.x = stod(result.at(0));
                point.y = stod(result.at(1));
                point.theta = stod(result.at(2));

                int pixel_x{0}, pixel_y{0};
                int end_pixel_x{0}, end_pixel_y{0};

                XYToPixel(pixel_x, pixel_y, point.x, point.y);
                XYToPixel(end_pixel_x,end_pixel_y,point.x+ 0.4*cos(point.theta), point.y + 0.4*sin(point.theta));

                cout << pixel_x << "  " << pixel_y << endl;

                auto arrow_start = cv::Point(pixel_x, pixel_y);
                auto arrow_end = cv::Point(end_pixel_x,end_pixel_y);

                //cv::line(img, cv::Point(1,1), cv::Point(1000, 1000), (255, 255, 255), 10 );
                cv::arrowedLine(img, arrow_start, arrow_end, (255, 255, 255), 1, 1, 0, 1);
            }
            
            //cv_bridge::CvImage img_bridge;
            //sensor_msgs::Image img_msg;
            //std_msgs::Header header;
            //img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
            //img_bridge.toImageMsg(img_msg);
            //pub.publish(img_msg);

            cv::imwrite("/home/lee/catkin_ws/src/zero/slam/src/global_path/global_path.png", img);
            //cv::waitKey(1);
            cout << 2 << endl;
        }

};

int main(int argc, char **argv){
    ros::init(argc, argv, "global_visual");
    GlobalVisual globalvisual;
    ros::spin();
    return 0;
}