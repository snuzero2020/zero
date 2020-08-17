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

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

    cloud->height = 1;
    cloud->width = left_seg_x.size();
    cloud->points.resize(left_seg_x.size());
    
    for(int i=0; i< cloud->size(); i++){
        cloud->points[i].x = msg->data[2*2*640*480 + left_seg_y[i]*640 + left_seg_x[i]];
        cloud->points[i].y = msg->data[2*2*640*480 + 640*480 + left_seg_y[i]*640 + left_seg_x[i]];
        cloud->points[i].z = msg->data[2*2*640*480 + 2*640*480 + left_seg_y[i]*640 + left_seg_x[i]];
    }

    std::cout << "PointCloud before filtering has: " << cloud->points.size () << " data points." << std::endl;
    
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(100);
    ec.setMaxClusterSize(5000);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    std::cout << "Number of clusters is equal to " << cluster_indices.size () << std::endl;

    cv::Mat left_cluster = cv::Mat::zeros(480, 640, CV_8UC1);

    int count = 1;

    for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end();++it){
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            left_cluster.at<uchar>(left_seg_y[*pit], left_seg_x[*pit]) = static_cast<int>(255/(count));
        }
        count++;
    }

    std::cout<<"C++ lane_postprocessing time : "<<(double)(clock()-start)/CLOCKS_PER_SEC<<std::endl;
    cv::imshow("left_cluster", left_cluster);
    cv::imshow("left_binary_seg", left_binary_seg);
    cv::imshow("right_binary_seg", right_binary_seg);
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