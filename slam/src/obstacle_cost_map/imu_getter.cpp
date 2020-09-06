#include <ros/ros.h>
#include <iostream>
//#include "utility.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#define PI 3.1415926535897
#define ANGLE 20 

using namespace std;

class ImuGetter
{
    public:
        ros::NodeHandle nh;
        ros::Publisher imu_raw_pub;
        ros::Subscriber imu_data_sub;

        sensor_msgs::Imu imu_data;
        //sensor_msgs::PointCloud2 velodyne_points;

    ImuGetter(){
        //imu_correct_pub = nh.advertise<sensor_msgs::Imu>("/imu_correct", 10);
        imu_raw_pub = nh.advertise<std_msgs::Float32>("/imu_raw", 10);
        imu_data_sub = nh.subscribe("/imu/data",100, &ImuGetter::imu_data_callback, this);
    }

    void imu_data_callback (const sensor_msgs::Imu::ConstPtr);
    //void imu_correct_publisher();
    void imu_raw_publisher();
};

void ImuGetter::imu_data_callback(const sensor_msgs::Imu::ConstPtr msg){
    
    imu_data.header.seq = msg->header.seq;
    imu_data.header.stamp = msg->header.stamp;
    imu_data.header.frame_id = msg->header.frame_id;
    
    imu_data.orientation.x = msg->orientation.x;
    imu_data.orientation.y = msg->orientation.y;
    imu_data.orientation.z = msg->orientation.z;
    imu_data.orientation.w = msg->orientation.w;

    imu_data.angular_velocity.x = msg->angular_velocity.x;
    imu_data.angular_velocity.y = msg->angular_velocity.y;
    imu_data.angular_velocity.z = msg->angular_velocity.z;

    imu_data.linear_acceleration.x = msg->linear_acceleration.x;
    imu_data.linear_acceleration.y = msg->linear_acceleration.y;
    imu_data.linear_acceleration.z = msg->linear_acceleration.z; 

    for (int i = 0; i < 9; ++i){
        imu_data.orientation_covariance[i] = msg->orientation_covariance[i];
        imu_data.angular_velocity_covariance[i] = msg->angular_velocity_covariance[i];
        imu_data.linear_acceleration_covariance[i] = msg->linear_acceleration_covariance[i];
    }

}

void ImuGetter::imu_raw_publisher(){
    tf::Quaternion q( imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z, imu_data.orientation.w );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    cout << "roll: " << roll << "\npitch: " << pitch << "\nyaw: " << yaw;
    std_msgs::Float32 imu_data;
    imu_data.data = pitch;
    imu_raw_pub.publish(imu_data);
}


int main(int argc, char *argv[])
{
    std::cout << "imu_getter started" << std::endl;  

	ros::init(argc,argv,"imu_getter");
	ros::NodeHandle nh;

	ImuGetter imugetter;
	ros::Rate loop_rate(10);

    while (ros::ok()){
        std::cout << 1 << std::endl;
        imugetter.imu_raw_publisher();
        ros::spinOnce();
        loop_rate.sleep();
    }

	return 0;
}