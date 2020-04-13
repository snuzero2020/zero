#include "ros/ros.h"
#include "ros/time.h"
#include "geometry_msgs/TwistStamped.h"
#include <opencv2/opencv.hpp>

#define WIDTH 1000
#define HEIGHT 1000
#define CYCLE 180

class GlobalMap{
public:
    GlobalMap(){
        sub_car_ = car_.subscribe("/global_position", 1, &GlobalMap::call_car, this);
        sub_cam_ = cam_.subscribe("/depth_camera", 1, &GlobalMap::call_cam, this);
    }
    void call_car(const geometry_msgs::TwistStamped::ConstPtr& msg){
        cur_car.header = msg->header;
        cur_car.twist = msg->twist;
    }
    void call_cam(const geometry_msgs::TwistStamped::ConstPtr& msg){
        ROS_INFO("subscribe cam : %.2f",msg->twist.angular.z);
        cur_cam.header = msg->header;
        cur_cam.twist = msg->twist;
    }
    geometry_msgs::TwistStamped get_car() { return cur_car; }
    geometry_msgs::TwistStamped get_cam() { return cur_cam; }
private:
    ros::NodeHandle car_;
    ros::NodeHandle cam_;
    ros::Subscriber sub_car_;
    ros::Subscriber sub_cam_;
    geometry_msgs::TwistStamped cur_car;
    geometry_msgs::TwistStamped cur_cam;
};
/*
void callback(const geometry_msgs::TwistStamped::ConstPtr& msg){
    cv::Mat global_map(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(255,255,255));
    //drawing car
    cv::RotatedRect car(cv::Point(msg->twist.linear.x, msg->twist.linear.y), cv::Size(20,10),msg->twist.angular.z * CYCLE / M_PI);
    cv::Point2f vertex_car_2f[4];
    cv::Point vertex_car[4];
    car.points(vertex_car_2f);
    for(int i=0;i<4;i++) vertex_car[i] = vertex_car_2f[i];
    cv::fillConvexPoly(global_map, vertex_car,4,cv::Scalar(255, 0, 0));
    
    //drawing depth camera
    //cv::rectangle(global_map, cv::Point(msg->twist.linear.x+50, msg->twist.linear.y+25), cv::Point(msg->twist.linear.x-50, msg->twist.linear.y-25), cv::Scalar(255, 0, 0));
    cv::imshow("map", global_map);
    cv::waitKey(1);
}
*/
int main(int argc, char **argv){
    ros::init(argc, argv, "show_car_position");
    GlobalMap GMobject;
    ros::Rate loop_rate(200);
    while(1){
        cv::Mat global_map(WIDTH, HEIGHT, CV_8UC3, cv::Scalar(255,255,255));
        
        //Drawing Car
        geometry_msgs::TwistStamped car = GMobject.get_car();
        cv::RotatedRect map_car(cv::Point(car.twist.linear.x, car.twist.linear.y), 
                                cv::Size(20,10),
                                car.twist.angular.z * CYCLE / M_PI);
        cv::Point2f vertex_car_2f[4];
        cv::Point vertex_car[4];
        map_car.points(vertex_car_2f);
        for(int i=0;i<4;i++) vertex_car[i] = vertex_car_2f[i];
        cv::fillConvexPoly(global_map, vertex_car, 4, cv::Scalar(255, 0, 0));

        //Drawing DepthCamera
        geometry_msgs::TwistStamped cam = GMobject.get_cam();
        cv::ellipse(global_map, cv::Point(car.twist.linear.x, car.twist.linear.y),
                    cv::Size(100,100),(car.twist.angular.z + cam.twist.angular.z) * CYCLE / M_PI,
                    -45, 45, cv::Scalar(0,0,255), CV_FILLED);

        //ROS_INFO("car position %.2f %.2f %.2f\ncam angle %.2f", car.twist.linear.x, car.twist.linear.y, car.twist.angular.z, cam.twist.angular.z);
        
        cv::imshow("map", global_map);
        cv::waitKey(10);
        ros::spinOnce();
        loop_rate.sleep();
    }
    /*
    ros::NodeHandle n;
    ros::Subscriber pose_sub = n.subscribe("/global_position", 1, callback);
    ros::spin();
    */
}