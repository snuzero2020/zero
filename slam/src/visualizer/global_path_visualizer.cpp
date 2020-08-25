#include <cmath>
#include <fstream>
#include <iostream>
#include <math.h>
#include <sstream>
#include <string>
#include <vector>

#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

#include "slam/Data.h"
#include "slam/GlobalPathPoint.h"
#include "slam/Pixel.h"

#include "ros/package.h"
#include "ros/ros.h"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "XYToPixel.h"

using namespace std;
using namespace cv;


class GlobalVisual{
    
    private:
        const char delimiter_ = ' '; 
        ros::NodeHandle nh;
        bool is_kcity;

    public:
		stringstream in_path_stream, in_map_stream, out_visual_stream;

        GlobalVisual(){
            ros::param::get("/is_kcity", is_kcity);

			if(is_kcity){
				in_path_stream << ros::package::getPath("slam") << "/config/KCity/global_path.txt";
				in_map_stream << ros::package::getPath("slam") << "/config/KCity/KCity.png";
				out_visual_stream << ros::package::getPath("slam")<< "/config/KCity/KCity_global_path_visual.png";
			}
			else{
				in_path_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_global_path.txt";
                in_map_stream << ros::package::getPath("slam") << "/config/FMTC/FMTC_map.png";
                out_visual_stream << ros::package::getPath("slam")<< "/config/FMTC/FMTC_global_path_visual.png";
			}
            global_path_visual();
        }

		
        void global_path_visual(){
            cv::Mat img = imread(in_map_stream.str(), IMREAD_COLOR);
			if (img.empty()){
                cout << "fuck" << endl;
            }
            cout  << 1 << endl;
            string in_line;
            ifstream in(in_path_stream.str());
			int i{1};
			bool flag_change_detect{false};
			int prev_flag{0};

            while(getline(in, in_line)){
                stringstream ss(in_line);
                string token;
                vector<string> result;
                printf("reading start ");
				while(getline(ss, token, delimiter_)){
                    result.push_back(token);
                }
				printf("loading start ");
                slam::GlobalPathPoint point;
                point.x = stod(result.at(0));
				printf("x loaded ");
                point.y = stod(result.at(1));
				printf("y loaded ");
                point.theta = stod(result.at(2));
				printf("theta loaded ");
				point.flag = stoi(result.at(3));
				printf("flag loaded\n");

                int pixel_x{0}, pixel_y{0};
                int end_pixel_x{0}, end_pixel_y{0};

                XYToPixel(pixel_x, pixel_y, point.x, point.y, is_kcity);
                XYToPixel(end_pixel_x,end_pixel_y,point.x+ 0.4*cos(point.theta), point.y + 0.4*sin(point.theta), is_kcity);

                cout << pixel_x << "  " << pixel_y << " " << i++ << endl;

                auto arrow_start = cv::Point(pixel_x, pixel_y);
                auto arrow_end = cv::Point(end_pixel_x,end_pixel_y);

				if(point.flag != prev_flag){
					flag_change_detect = !flag_change_detect;
					prev_flag = point.flag;
				}
				if(flag_change_detect) cv::circle(img, arrow_start, 2, cv::Scalar(255,0,0), -1);
				else cv::circle(img, arrow_start, 2, cv::Scalar(0,255,0), -1);
				/*if(i%2 != 0) cv::putText(img, std::to_string(i++), arrow_start, 1, 0.5, cv::Scalar::all(255));
				else i++;
				-----------for checking number of  outliers by visualizing-------------------------
				*/
				//cv::line(img, cv::Point(1,1), cv::Point(1000, 1000), (255, 255, 255), 10 );
                //cv::arrowedLine(img, arrow_start, arrow_end, (255, 255, 255), 1, 1, 0, 1);
			}
			printf("out of while");
           


            //cv_bridge::CvImage img_bridge;
            //sensor_msgs::Image img_msg;
            //std_msgs::Header header;
            //img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
            //img_bridge.toImageMsg(img_msg);
            //pub.publish(img_msg);
            cv::imwrite(out_visual_stream.str(), img);
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
