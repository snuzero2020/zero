#include "ros/ros.h"
#include "ros/time.h"
#include "opencv2/opencv.hpp"
#include <iostream>
#include <vector>
#include "slam/Yoloinfo.h"
#include "slam/imgCluster.h"
#include "geometry_msgs/Point.h"
#include "math.h"

using namespace std;
using namespace cv;

class YoloFusion{
    public:
    YoloFusion(){
        pcl_sub_ = nh_.subscribe("/image_lidar",1, &YoloFusion::lidarcallback, this);
        yolo_sub_ = nh_.subscribe("/yolo_info",1, &YoloFusion::yolocallback, this);
    }

    void lidarcallback(const slam::imgCluster::ConstPtr& msg){
        vector_initializer();
        cluster_lidar = msg->clusters;
        points_lidar = msg->points;
    }

    void yolocallback(const slam::Yoloinfo::ConstPtr& msg){
        label_box = msg->label;
        width_box = msg->width;
        height_box = msg->height;
        points_box = msg->points;

        // for(int i=0; i<label_box.size(); i++){
        //     for(int j; j<msg->count; j++)
        // }
    }

    bool isIncluded(int x, int y, int index){
        int w = width_box.at(index);
        int h = height_box.at(index);
        int px = points_box.at(index).x;
        int py = points_box.at(index).y;
        if((x <= px + w/2) && (x >= px - w/2) && (y <= py + h/2) && (y >= py - h/2)){
            is_included.push_back(true);
        }
    }

    //Initialize all the vectors
    void vector_initializer(){
        points_lidar.clear();
        cluster_lidar.clear();
        label_box.clear();
        width_box.clear();
        height_box.clear();
        points_box.clear();
        is_included.clear();
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber yolo_sub_;
    ros::Subscriber pcl_sub_;

    //PROJECTED_LIDAR_VECTORS
    vector<geometry_msgs::Point> points_lidar; // points of projected LiDar
    vector<int> cluster_lidar; // cluster of projected LiDar
    vector<bool> is_included; // Existence of projected LiDar

    //YOLO_BOX_VECTORS
    vector<int> label_box; // label of yolo box
    vector<int> width_box; // width of yolo box
    vector<int> height_box; // height of yolo box
    vector<geometry_msgs::Point> points_box; // center of yolo box

    //SORTED_BY_CLUSTER_NUMBER
    vector<double> p_cluster; // Possibility that cluster be included in yolo box
    vector<int> combi_cluster; // cluster's architecture
};

int main(int argc, char **argv){
    ros::init(argc, argv, "yolo_lidar");
    //YoloFusion yolofusion;
    ros::spin();
}
