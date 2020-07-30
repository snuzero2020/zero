#include "ros/ros.h"
#include "ros/console.h"
#include "slam/Gps.h"
#include "slam/Data.h"
#include "slam/Imu.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include <cmath>
#include "XYToPixel.h"
#include "ros/package.h"
using namespace std;

#define REF_X 298500
#define REF_Y 4137850

class Map_Marker{
    public:
    Map_Marker(){
        sub_gps = n_.subscribe("/gps", 10000, &Map_Marker::callback_gps, this);
        sub_filtered = n_.subscribe("/filtered_data", 10000, &Map_Marker::callback_filtered, this);
        sub_imu = n_.subscribe("/imu", 10000, &Map_Marker::callback_imu, this);
        path_stream << ros::package::getPath("slam") << "/src/mapping/map.png";
        img = cv::imread(path_stream.str(), 1);
        ROS_INFO("Image loaded");
    }

    void callback_gps(const slam::Gps::ConstPtr& msg){
	    int pixel_x, pixel_y;

        XYToPixel(pixel_x, pixel_y, msg->x, msg->y);
        if (n != 1) {
            cv::line(img, cv::Point(prev_pixel_x, prev_pixel_y), cv::Point(pixel_x, pixel_y), cv::Scalar(0, 0, 255), 3);
            cv::circle(img, cv::Point(filtered_pixel_x, filtered_pixel_y), 3, cv::Scalar(255, 0, 0), -1);
            if( n % 40 == 0){
                //cv::arrowedLine(img, cv::Point(filtered_pixel_x, filtered_pixel_y), cv::Point(filtered_pixel_x+2*filtered_pixel_vx, filtered_pixel_y+2*filtered_pixel_vy), cv::Scalar(255, 0, 255), 16, 8, 0, 0.2);
                cv::arrowedLine(img, cv::Point(filtered_pixel_x, filtered_pixel_y), cv::Point(filtered_pixel_x+2*filtered_pixel_thx, pixel_y+2*filtered_pixel_thy), cv::Scalar(255, 255, 0), 10, 8, 0, 0.5);
                //cv::arrowedLine(img, cv::Point(pixel_x, pixel_y), cv::Point(pixel_x+2*mag_pixel_thx, pixel_y+2*mag_pixel_thy), cv::Scalar(255, 255, 0), 16, 8, 0, 0.2);
            }
        }
        t = ros::Time::now();
        saved = false;
        prev_pixel_x = pixel_x;
        prev_pixel_y = pixel_y;

        ROS_DEBUG_STREAM("gps_marker: GPS: position(" << msg->x << ", " << msg->y << ") -> position(" << pixel_x << ", " << pixel_y << ")");
        n++;
    }

    void callback_filtered(const slam::Data::ConstPtr& msg){
	    int pixel_x, pixel_y;

        XYToPixel(filtered_pixel_x, filtered_pixel_y, msg->x, msg->y);
        XYToPixel(pixel_x, pixel_y, REF_X, REF_Y);
        XYToPixel(filtered_pixel_vx, filtered_pixel_vy, REF_X+msg->vx, REF_Y+msg->vy);
        filtered_pixel_vx -= pixel_x;
        filtered_pixel_vy -= pixel_y;
        XYToPixel(filtered_pixel_thx, filtered_pixel_thy, REF_X+2*cos(msg->theta), REF_Y+2*sin(msg->theta));
        filtered_pixel_thx -= pixel_x;
        filtered_pixel_thy -= pixel_y;

        ROS_DEBUG_STREAM("gps_marker: Kalman-filter: (" << msg->x << ", " << msg->y << ") -> position(" << pixel_x << ", " << pixel_y << \
        "), velocity(" << filtered_pixel_vx << ", " << filtered_pixel_vy << "), theta(" << filtered_pixel_thx << ", " << filtered_pixel_thy << ")");
    }

    void callback_imu(const slam::Imu::ConstPtr& msg){
	    int pixel_x, pixel_y;

        XYToPixel(pixel_x, pixel_y, REF_X, REF_Y);
        XYToPixel(mag_pixel_thx, mag_pixel_thy, REF_X+2*cos(msg->theta), REF_Y+2*sin(msg->theta));
        mag_pixel_thx -= pixel_x;
        mag_pixel_thy -= pixel_y;

        ROS_DEBUG_STREAM("gps_marker: IMU: theta(" << msg->theta << ") -> position(" << pixel_x << ", " << pixel_y << ")");
    }

    void save_if_end(){
        if( n!=1 && saved==false && (ros::Time::now()-t).sec > 4 ){
            path_stream.str(std::string());
            path_stream << ros::package::getPath("slam") << "/src/mapping/path" << n << ".png";
            cv::imwrite(path_stream.str(), img);
            ROS_INFO("Image saved %d",n);
            saved = true;
        }
    }

    private:
    ros::NodeHandle n_;
    ros::Subscriber sub_gps;
    ros::Subscriber sub_filtered;
    ros::Subscriber sub_imu;
    ros::Subscriber sub_keyboard;
    cv::Mat img;
    stringstream path_stream;
    int n = 1;
    int prev_pixel_x = 0;
    int prev_pixel_y = 0;
    int filtered_pixel_x = -1;
    int filtered_pixel_y = -1;
    int filtered_pixel_vx = -1;
    int filtered_pixel_vy = -1;
    int filtered_pixel_thx = -1;
    int filtered_pixel_thy = -1;
    int mag_pixel_thx = -1;
    int mag_pixel_thy = -1;
    bool saved = false;
    ros::Time t;
};

int main(int argc, char** argv) {
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
        ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "map_marker");
    Map_Marker map_marker;
    while (ros::ok()){
        map_marker.save_if_end();
        ros::spinOnce();
    }
}