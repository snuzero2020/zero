#include <ros/ros.h>
#include <iostream>
//#include "utility.h"
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <queue>
#include <vector>
#include <algorithm>

#define PI 3.1415926535897
#define ANGLE 20 

using namespace std;

class GetPitch
{
    public:
        ros::NodeHandle nh;
        ros::Publisher pub;
        ros::Subscriber sub;
        queue<double> qu;
        
    GetPitch(){
        pub = nh.advertise<std_msgs::Float32>("/imu_raw", 10);
        sub = nh.subscribe("/imu/data",100, &GetPitch::callback, this);
    }

    void callback (const sensor_msgs::Imu::ConstPtr);
};

void GetPitch::callback(const sensor_msgs::Imu::ConstPtr msg){
    
    tf::Quaternion q(msg->orientation.x,msg->orientation.y,msg->orientation.z,msg->orientation.w);
    tf::Matrix3x3 m(q);
    double r,p,y;
    m.getRPY(r,p,y);
    qu.push(p);
    if(qu.size()<=10) return;
    qu.pop();
    queue<double> tmp = qu;
    vector<double> query;
    while(!tmp.empty()){
        query.push_back(tmp.front());
        tmp.pop();
    }
    sort(query.begin(), query.end());

    double sum = 0;
    for(int i=2;i<8;i++){
        sum+= query[i];
    }
    sum /= 6.0;
    std_msgs::Float32 rt;
    rt.data = sum;
    pub.publish(rt);
}



int main(int argc, char *argv[])
{
    ros::init(argc,argv,"get_pitch");
	GetPitch get_pitch;
	ros::spin();
}