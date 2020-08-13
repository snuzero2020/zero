#include "ros/ros.h"
#include "ros/time.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include "Eigen/Eigen"
#include <vector>
#include <string>
//Message Type
#include "slam/imgCluster.h"
#include "geometry_msgs/Point.h"
#include "slam/Clustermaster.h"

using namespace cv;
using namespace std;

class Visualizer{
    public:
    Visualizer(){
        sub_ = nh_.subscribe("/pcl_on_image", 1, &Visualizer::callback, this);
        color_randomizer();
    }

    void callback(const slam::Clustermaster::ConstPtr& msg){
        Mat img = imread("/home/jeongwoooh/vision/signa100557.jpg",IMREAD_COLOR);
        lidar_on_image = msg->clusters;
        for(slam::imgCluster cluster : lidar_on_image){
            int color_flag = 0;
            for(geometry_msgs::Point pt : cluster.points){
                if(pt.x <= img.rows && pt.y <= img.cols && pt.x >=0 && pt.y >= 0){
                    img.at<Vec3b>(pt.y, pt.x) = color_list.at(color_flag % 3); 
                }
                else cout << "Out of range" << endl;
            }
            color_flag += 1;
        }
        imwrite("/home/jeongwoooh/vision/lidar_projected.jpg", img);
	printf("completed to save img, img size : %d \n", img.size());
    }

    void color_randomizer(){
        color_list.push_back(Vec3b(255,0,0)); //Blue
        color_list.push_back(Vec3b(0,255,0)); //Green
        color_list.push_back(Vec3b(0,0,255)); //Red
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    vector<Vec3b> color_list;
    vector<slam::imgCluster> lidar_on_image;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_visualizer");
    Visualizer visualizer;
    ros::spin();
}
