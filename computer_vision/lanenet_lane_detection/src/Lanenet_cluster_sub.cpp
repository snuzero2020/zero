#include "ros/ros.h"
#include <iostream>
#include "lanenet_lane_detection/lanenet_clus_msg.h"
#include <opencv2/opencv.hpp>
#include <ctime>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace cv;
using namespace std;

void lanenet_callback(const lanenet_lane_detection::lanenet_clus_msg::ConstPtr &msg){
    clock_t start;
    start = clock();
    
    cv::Mat left_binary_seg = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat right_binary_seg = cv::Mat::zeros(480, 640, CV_8UC1);
    vector<int> left_seg_x, right_seg_x;
    vector<int> left_seg_y, right_seg_y;

    for(int h=0; h<480; h++){
        for(int w=0; w<640; w++){
            if(msg->data[h*640 + w] < msg->data[480*640 + h*640 + w]){
                left_binary_seg.at<uchar>(h,w) = 255;
                left_seg_x.push_back(w);
                left_seg_y.push_back(h);
            }
            if(msg->data[2*640*480 + h*640 + w] < msg->data[2*640*480 + 480*640 + h*640 + w]){
                right_binary_seg.at<uchar>(h,w) = 255;
                right_seg_x.push_back(w);
                right_seg_y.push_back(h);
            }
        }
    }

    // left_img clustering

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>);
    
    cloud_left->height = 1;
    cloud_left->width = left_seg_x.size();
    cloud_left->points.resize(left_seg_x.size());
    
    cloud_right->height = 1;
    cloud_right->width = right_seg_x.size();
    cloud_right->points.resize(right_seg_x.size());
    
    
    for(int i=0; i< cloud_left->size(); i++){
        cloud_left->points[i].x = msg->data[2*2*640*480 + left_seg_y[i]*640 + left_seg_x[i]];
        cloud_left->points[i].y = msg->data[2*2*640*480 + 640*480 + left_seg_y[i]*640 + left_seg_x[i]];
        cloud_left->points[i].z = msg->data[2*2*640*480 + 2*640*480 + left_seg_y[i]*640 + left_seg_x[i]];
    }
    for(int i=0; i< cloud_right->size(); i++){
        cloud_right->points[i].x = msg->data[2*2*640*480 + 3*640*480 + right_seg_y[i]*640 + right_seg_x[i]];
        cloud_right->points[i].y = msg->data[2*2*640*480 + 3*640*480 + 640*480 + right_seg_y[i]*640 + right_seg_x[i]];
        cloud_right->points[i].z = msg->data[2*2*640*480 + 3*640*480 + 2*640*480 + right_seg_y[i]*640 + right_seg_x[i]];
    }
    
    std::cout<<"check"<<std::endl;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr left_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr right_tree (new pcl::search::KdTree<pcl::PointXYZ>);

    left_tree->setInputCloud(cloud_left);
    right_tree->setInputCloud(cloud_right);

    std::vector<pcl::PointIndices> left_cluster_indices;
    std::vector<pcl::PointIndices> right_cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_left;
    ec_left.setClusterTolerance(0.034);
    ec_left.setMinClusterSize(100);
    ec_left.setMaxClusterSize(5000);
    ec_left.setSearchMethod(left_tree);
    ec_left.setInputCloud(cloud_left);
    ec_left.extract(left_cluster_indices);

    cv::Mat left_cluster = cv::Mat::zeros(480, 640, CV_8UC1);

    int count = 1;

    for(std::vector<pcl::PointIndices>::const_iterator it = left_cluster_indices.begin();it != left_cluster_indices.end();++it){
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            left_cluster.at<uchar>(left_seg_y[*pit], left_seg_x[*pit]) = static_cast<int>(255/(count));
        }
        count++;
    }
    
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_right;
    ec_right.setClusterTolerance(0.034);
    ec_right.setMinClusterSize(100);
    ec_right.setMaxClusterSize(5000);;
    ec_right.setSearchMethod(right_tree);
    ec_right.setInputCloud(cloud_right);
    ec_right.extract(right_cluster_indices);
    
    cv::Mat right_cluster = cv::Mat::zeros(480, 640, CV_8UC1);

    count = 1;

    for(std::vector<pcl::PointIndices>::const_iterator it = right_cluster_indices.begin();it != right_cluster_indices.end();++it){
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            right_cluster.at<uchar>(right_seg_y[*pit], right_seg_x[*pit]) = static_cast<int>(255/(count));
        }
        count++;
    }

    std::cout<<"C++ lane_postprocessing time : "<<(double)(clock()-start)/CLOCKS_PER_SEC<<std::endl;

    cv::imshow("right_cluster", right_cluster);
    cv::imshow("left_cluster", left_cluster);
    //cv::imshow("left_binary_seg", left_binary_seg);
    //cv::imshow("right_binary_seg", right_binary_seg);
    cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lanenet_cluster_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber lanenet_sub = nh.subscribe("/lane_cluster_topic",100, lanenet_callback);

    ros::spin();

    return 0;
}