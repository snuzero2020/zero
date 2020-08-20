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
#include "multi_img_utils.h"
#include <cmath>

using namespace cv;
using namespace std;

int x_and_y_pow(int x, int x_n, int y, int y_n)
{
    if(x_n == 0 && y_n !=0)
    {
        return pow(y, y_n);
    }
    else if(x_n != 0 && y_n ==0)
    {
        return pow(x, x_n);
    }
    else
    {
        return pow(x, x_n)*pow(y, y_n);
    }
}

cv::Mat find_birdmat_left(){
    cv::Point2f src_vertices[6];
    cv::Point2f dst_vertices[6];

    src_vertices[0]=cv::Point2f(262, 168);
	src_vertices[1]=cv::Point2f(308, 0);
	src_vertices[2]=cv::Point2f(480, 0);
	src_vertices[3]=cv::Point2f(480, 640);
	src_vertices[4]=cv::Point2f(269, 640);
	src_vertices[5]=cv::Point2f(260, 590);

    dst_vertices[0]=cv::Point2f(0, 0);
	dst_vertices[1]=cv::Point2f(119, 0);
	dst_vertices[2]=cv::Point2f(207, 63);
	dst_vertices[3]=cv::Point2f(204, 116);
	dst_vertices[4]=cv::Point2f(43, 200);
	dst_vertices[5]=cv::Point2f(0, 200);

    cv::Mat M_left = cv::getPerspectiveTransform(src_vertices, dst_vertices);
    return M_left;
} 

cv::Mat find_birdmat_right(){
    cv::Point2f src_vertices[6];
    cv::Point2f dst_vertices[6];

    src_vertices[0]=cv::Point2f(270, 70);
	src_vertices[1]=cv::Point2f(286, 0);
	src_vertices[2]=cv::Point2f(480, 0);
	src_vertices[3]=cv::Point2f(480, 640);
	src_vertices[4]=cv::Point2f(294, 640);
	src_vertices[5]=cv::Point2f(260, 486);

    dst_vertices[0]=cv::Point2f(0, 0);
	dst_vertices[1]=cv::Point2f(55, 0);
	dst_vertices[2]=cv::Point2f(203, 80);
	dst_vertices[3]=cv::Point2f(205, 134);
	dst_vertices[4]=cv::Point2f(108, 200);
	dst_vertices[5]=cv::Point2f(0, 200);

    cv::Mat M_right = cv::getPerspectiveTransform(src_vertices, dst_vertices);
    return M_right;
}

void birdeye(vector<int> one_indice, cv::Mat birdmat, vector<vector<int>> &cluster_indices){
    vector<int> bird_indice;
    double z_double = birdmat.at<double>(2,0)*one_indice[0]+birdmat.at<double>(2,1)*one_indice[1]+birdmat.at<double>(2,2)*one_indice[2];
    bird_indice.push_back(static_cast<int>((birdmat.at<double>(0,0)*one_indice[0]+birdmat.at<double>(0,1)*one_indice[1]+birdmat.at<double>(0,2)*one_indice[2])/z_double));
    bird_indice.push_back(static_cast<int>((birdmat.at<double>(1,0)*one_indice[0]+birdmat.at<double>(1,1)*one_indice[1]+birdmat.at<double>(1,2)*one_indice[2])/z_double));
    if(bird_indice[0]<267 && bird_indice[0]>=0 && bird_indice[1]<200 && bird_indice[1]>=0){
                cluster_indices.push_back(bird_indice);
    }
}

bool is_birdeye_included(cv::Mat birdmat, int y, int x){
    double z_double = birdmat.at<double>(2,0)*y+birdmat.at<double>(2,1)*x+birdmat.at<double>(2,2);
    int y_new = static_cast<int>((birdmat.at<double>(0,0)*y+birdmat.at<double>(0,1)*x+birdmat.at<double>(0,2))/z_double);
    int x_new = static_cast<int>((birdmat.at<double>(1,0)*y+birdmat.at<double>(1,1)*x+birdmat.at<double>(1,2))/z_double);
    if(y_new >=0 && y_new < 267 && x_new >= 0 && x_new < 200){
        return true;
    }
    return false;
}

void getcoeff(vector<vector<int>> clus_indices, double* coeff){
    double A[4][4];
    double B[4];
    int size = clus_indices.size();

    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            A[i][j] = 0;
        }
    }
    
    A[0][0] = static_cast<double>(size);
    for(int i=0; i<size; i++)
    {
        A[0][1] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 1));
        A[1][0] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 1));

        A[0][2] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 2));
        A[1][1] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 2));
        A[2][0] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 2));

        A[0][3] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 3));
        A[1][2] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 3));
        A[2][1] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 3));
        A[3][0] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 3));
    
        A[1][3] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 4));
        A[2][2] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 4));
        A[3][1] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 4));

        A[2][3] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 5));
        A[3][2] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 5));

        A[3][3] += static_cast<double>(x_and_y_pow(0, 0, clus_indices[i][0], 6));

        B[0] += static_cast<double>(x_and_y_pow(clus_indices[i][1], 1, 0, 0));
        B[1] += static_cast<double>(x_and_y_pow(clus_indices[i][1], 1, clus_indices[i][0], 1));
        B[2] += static_cast<double>(x_and_y_pow(clus_indices[i][1], 1, clus_indices[i][0], 2));
        B[3] += static_cast<double>(x_and_y_pow(clus_indices[i][1], 1, clus_indices[i][0], 3));
    }

    double inverse_A[4][4];

    inverse_A[0][0] = A[1][1]*A[2][2]*A[3][3]+A[1][2]*A[2][3]*A[3][1]+A[1][3]*A[2][1]*A[3][2]-A[1][3]*A[2][2]*A[3][1]-A[1][2]*A[2][1]*A[3][3]-A[1][1]*A[2][3]*A[3][2];
    inverse_A[1][0] = inverse_A[0][1] = A[0][3]*A[2][2]*A[3][1]+A[0][2]*A[2][1]*A[3][3]+A[0][1]*A[2][3]*A[3][2]-A[0][1]*A[2][2]*A[3][3]-A[0][2]*A[2][3]*A[3][1]-A[0][3]*A[2][1]*A[3][2];
    inverse_A[2][0] = inverse_A[0][2] = A[0][1]*A[1][2]*A[3][3]+A[0][2]*A[1][3]*A[3][1]+A[0][3]*A[1][1]*A[3][2]-A[0][3]*A[1][2]*A[3][1]-A[0][2]*A[1][1]*A[3][3]-A[0][1]*A[1][3]*A[3][2];
    inverse_A[0][3] = inverse_A[3][0] = A[0][3]*A[1][2]*A[2][1]+A[0][2]*A[1][1]*A[2][3]+A[0][1]*A[1][3]*A[2][2]-A[0][1]*A[1][2]*A[2][3]-A[0][2]*A[1][3]*A[2][1]-A[0][3]*A[1][1]*A[2][2];
    inverse_A[1][1] = A[0][0]*A[2][2]*A[3][3]+A[0][2]*A[2][3]*A[3][0]+A[0][3]*A[2][0]*A[3][2]-A[0][3]*A[2][2]*A[3][0]-A[0][2]*A[2][0]*A[3][3]-A[0][0]*A[2][3]*A[3][2];
    inverse_A[2][1] = inverse_A[1][2] = A[0][3]*A[1][2]*A[3][0]+A[0][2]*A[1][0]*A[3][3]+A[0][0]*A[1][3]*A[3][2]-A[0][0]*A[1][2]*A[3][3]-A[0][2]*A[1][3]*A[3][0]-A[0][3]*A[1][0]*A[3][2];
    inverse_A[3][1] = inverse_A[1][3] = A[0][0]*A[1][2]*A[2][3]+A[0][2]*A[1][3]*A[2][0]+A[0][3]*A[1][0]*A[2][2]-A[0][3]*A[1][2]*A[2][0]-A[0][2]*A[1][0]*A[2][3]-A[0][0]*A[1][3]*A[2][2];
    inverse_A[2][2] = A[0][0]*A[1][1]*A[3][3]+A[0][1]*A[1][3]*A[3][0]+A[0][3]*A[1][0]*A[3][1]-A[0][3]*A[1][1]*A[3][0]-A[0][1]*A[1][0]*A[3][3]-A[0][0]*A[1][3]*A[3][1];
    inverse_A[3][2] = inverse_A[2][3] = A[0][3]*A[1][1]*A[2][0]+A[0][1]*A[1][0]*A[2][3]+A[0][0]*A[1][3]*A[2][1]-A[0][0]*A[1][1]*A[2][3]-A[0][1]*A[1][3]*A[2][0]-A[0][3]*A[1][0]*A[2][1];
    inverse_A[3][3] = A[0][0]*A[1][1]*A[2][2]+A[0][1]*A[1][2]*A[2][0]+A[0][2]*A[1][0]*A[2][1]-A[0][2]*A[1][1]*A[2][0]-A[0][1]*A[1][0]*A[2][2]-A[0][0]*A[1][2]*A[2][1];

    double det_a = 0;
    det_a += A[0][0]*(A[1][1]*A[2][2]*A[3][3]+A[1][2]*A[2][3]*A[3][1]+A[1][3]*A[2][1]*A[3][2]-A[1][3]*A[2][2]*A[3][1]-A[1][2]*A[2][1]*A[3][3]-A[1][1]*A[2][3]*A[3][2]);
    det_a -= A[1][0]*(A[0][1]*A[2][2]*A[3][3]+A[0][2]*A[2][3]*A[3][1]+A[0][3]*A[2][1]*A[3][2]-A[0][3]*A[2][2]*A[3][1]-A[0][2]*A[2][1]*A[3][3]-A[0][1]*A[2][3]*A[3][2]);
    det_a += A[2][0]*(A[0][1]*A[1][2]*A[3][3]+A[0][2]*A[1][3]*A[3][1]+A[0][3]*A[1][1]*A[3][2]-A[0][3]*A[1][2]*A[3][1]-A[0][2]*A[1][1]*A[3][3]-A[0][1]*A[1][3]*A[3][2]);
    det_a -= A[3][0]*(A[0][1]*A[1][2]*A[2][3]+A[0][2]*A[1][3]*A[2][1]+A[0][3]*A[1][1]*A[2][2]-A[0][3]*A[1][2]*A[2][1]-A[0][2]*A[1][1]*A[2][3]-A[0][1]*A[1][3]*A[2][2]);

    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            inverse_A[i][j] /= det_a;
        }
    }

    coeff[0] = inverse_A[0][0]*B[0] + inverse_A[0][1]*B[1] + inverse_A[0][2]*B[2] + inverse_A[0][3]*B[3];
    coeff[1] = inverse_A[1][0]*B[0] + inverse_A[1][1]*B[1] + inverse_A[1][2]*B[2] + inverse_A[1][3]*B[3];
    coeff[2] = inverse_A[2][0]*B[0] + inverse_A[2][1]*B[1] + inverse_A[2][2]*B[2] + inverse_A[2][3]*B[3];
    coeff[3] = inverse_A[3][0]*B[0] + inverse_A[3][1]*B[1] + inverse_A[3][2]*B[2] + inverse_A[3][3]*B[3];

    // std::cout<<coeff[0]<<std::endl;
    // std::cout<<coeff[1]<<std::endl;
    // std::cout<<coeff[2]<<std::endl;
    // std::cout<<coeff[3]<<std::endl;
}

void lanenet_callback(const lanenet_lane_detection::lanenet_clus_msg::ConstPtr &msg){
    clock_t start;
    start = clock();
    
    //cv::Mat left_binary_seg = cv::Mat::zeros(480, 640, CV_8UC1);
    //cv::Mat right_binary_seg = cv::Mat::zeros(480, 640, CV_8UC1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_left(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_right(new pcl::PointCloud<pcl::PointXYZ>);
    
    vector<int> left_seg_x, right_seg_x;
    vector<int> left_seg_y, right_seg_y;
    
    cv::Mat birdmat_left = find_birdmat_left();
    cv::Mat birdmat_right = find_birdmat_right();

    for(int h=0; h<480; h++){
        for(int w=0; w<640; w++){
            if(msg->data[h*640 + w] < msg->data[480*640 + h*640 + w]){
                //left_binary_seg.at<uchar>(h,w) = 255;
                if(is_birdeye_included(birdmat_left, h, w)){
                    left_seg_x.push_back(w);
                    left_seg_y.push_back(h);
                }
            }
            if(msg->data[2*640*480 + h*640 + w] < msg->data[2*640*480 + 480*640 + h*640 + w]){
                //right_binary_seg.at<uchar>(h,w) = 255;
                if(is_birdeye_included(birdmat_right, h, w)){
                    right_seg_x.push_back(w);
                    right_seg_y.push_back(h);
                }
            }
        }
    }
    //std::cout<<left_seg_x.size()<<std::endl;

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
    };

    pcl::search::KdTree<pcl::PointXYZ>::Ptr left_tree (new pcl::search::KdTree<pcl::PointXYZ>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr right_tree (new pcl::search::KdTree<pcl::PointXYZ>);

    left_tree->setInputCloud(cloud_left);
    right_tree->setInputCloud(cloud_right);

    std::vector<pcl::PointIndices> left_cluster_indices;
    std::vector<pcl::PointIndices> right_cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_left;
    ec_left.setClusterTolerance(0.15);
    ec_left.setMinClusterSize(10);
    ec_left.setMaxClusterSize(5000);
    ec_left.setSearchMethod(left_tree);
    ec_left.setInputCloud(cloud_left);
    ec_left.extract(left_cluster_indices);

    // cv::Mat left_cluster = cv::Mat::zeros(480, 640, CV_8UC1);

    int count = 1;
    vector<vector<int>> cluster_indices;
    vector<vector<vector<int>>> all_indices;



    for(std::vector<pcl::PointIndices>::const_iterator it = left_cluster_indices.begin();it != left_cluster_indices.end();++it){
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            // left_cluster.at<uchar>(left_seg_y[*pit], left_seg_x[*pit]) = static_cast<int>(255/(count));
            vector<int> one_indice;
            one_indice.push_back(left_seg_y[*pit]);
            one_indice.push_back(left_seg_x[*pit]);
            one_indice.push_back(1);
            birdeye(one_indice, birdmat_left, cluster_indices);
        }
        all_indices.push_back(cluster_indices);
        cluster_indices.clear();
        count++;
    }

    int left_lane = 0;
    double center[2] = {0, 0};

    for(int i = 0; i<2; i++){
        for(int j = 0; j<all_indices[i].size(); j++){
            center[i] += all_indices[i][j][1];
        }
        center[i]/=all_indices[i].size();
    }

    if(center[0]<center[1])
        left_lane = 0;
    else
        left_lane = 1;


    cv::Mat birdeye_img = cv::Mat::zeros(267, 200, CV_8UC1);

    for(int j = 0; j<all_indices[left_lane].size(); j++){
        birdeye_img.at<uchar>(all_indices[left_lane][j][0], all_indices[left_lane][j][1]) = 255;
    }

    double coeff_left[4];
    getcoeff(all_indices[left_lane], coeff_left);

    // std::cout << coeff_left[0] <<" "<<coeff_left[1] << " " << coeff_left[2] << " " << coeff_left[3] << std::endl;
    
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec_right;

    ec_right.setClusterTolerance(0.15);
    ec_right.setMinClusterSize(10);
    ec_right.setMaxClusterSize(5000);;
    ec_right.setSearchMethod(right_tree);
    ec_right.setInputCloud(cloud_right);
    ec_right.extract(right_cluster_indices);
    
    // cv::Mat right_cluster = cv::Mat::zeros(480, 640, CV_8UC1);

    count = 1;
    cluster_indices.clear();
    all_indices.clear();

    for(std::vector<pcl::PointIndices>::const_iterator it = right_cluster_indices.begin();it != right_cluster_indices.end();++it){
        for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit){
            // right_cluster.at<uchar>(right_seg_y[*pit], right_seg_x[*pit]) = static_cast<int>(255/(count));
            vector<int> one_indice;
            one_indice.push_back(right_seg_y[*pit]);
            one_indice.push_back(right_seg_x[*pit]);
            one_indice.push_back(1);
            birdeye(one_indice, birdmat_right, cluster_indices);
            
        }
        all_indices.push_back(cluster_indices);
        cluster_indices.clear();
        count++;
    }

    int right_lane = 0;
    center[0] = 0;
    center[1] = 0;

    for(int i = 0; i<2; i++){
        for(int j = 0; j<all_indices[i].size(); j++){
            center[i] += all_indices[i][j][1];
        }
        center[i]/=all_indices[i].size();
    }

    if(center[0]>center[1])
        right_lane = 0;
    else
        right_lane = 1;

    for(int j = 0; j<all_indices[right_lane].size(); j++){
        birdeye_img.at<uchar>(all_indices[right_lane][j][0], all_indices[right_lane][j][1]) = 100;
    }

    double coeff_right[4];
    getcoeff(all_indices[right_lane], coeff_right);

    //cv::imshow("birdeye_leftnright_img", birdeye_img);

    cv::waitKey(1);
    // std::cout << all_indices.size() << std::endl;

    //cv::Mat fitting_img = cv::Mat::zeros(200, 200, CV_8UC1);
    cv::Mat costMap = cv::Mat::zeros(267, 200, CV_8UC1);

    int j_left, j_right;

    for(int i=0; i<267; i++){
        j_left = coeff_left[0]+coeff_left[1]*i+coeff_left[2]*i*i+coeff_left[3]*i*i*i;
        j_right = coeff_right[0]+coeff_right[1]*i+coeff_right[2]*i*i+coeff_right[3]*i*i*i;
        int fifth = (j_right-j_left)/5;
        /*for(int j = j_left-2; j<j_left; j++){
            if(j<200 && j>=0)
                fitting_img.at<uchar>(i,j) = 255;
        }
        for(int j = j_right+1; j<j_right+3; j++){
            if(j<200 && j>=0)
                fitting_img.at<uchar>(i,j) = 255;
        }*/
        for(int j = j_left; j<j_left+fifth; j++){
            if(j<200 && j>=0)
                costMap.at<uchar>(i,j) = 200;
        }
        for(int j = j_left+fifth; j<j_left+2*fifth; j++){
            if(j<200 && j>=0)
                costMap.at<uchar>(i,j) = 100;
        }
        for(int j = j_left+2*fifth; j<j_right-2*fifth; j++){
            if(j<200 && j>=0)
                costMap.at<uchar>(i,j) = 0;
        }
        for(int j = j_right-2*fifth; j<j_right-fifth; j++){
            if(j<200 && j>=0)
                costMap.at<uchar>(i,j) = 100;
        }
        for(int j = j_right-fifth; j<=j_right; j++){
            if(j<200 && j>=0)
                costMap.at<uchar>(i,j) = 200;
        }
    }

    std::cout << "Goal point " << (j_left+j_right)/2 << std::endl;

    //cv::imshow("fitting_img", fitting_img);
    //cv::imshow("costMap", costMap);

    std::cout<<"C++ lane_postprocessing time : "<<(double)(clock()-start)/CLOCKS_PER_SEC<<std::endl;
    //ShowManyImages("Cluster_image", 2, birdeye_img, costMap);

    cv::imshow("birdeye_img", birdeye_img);
    cv::imshow("costMap", costMap);

    //cv::imshow("right_cluster", right_cluster);
    //cv::imshow("left_cluster", left_cluster);
    // ShowManyImages("Cluster_image",2, left_cluster, right_cluster);
    // cv::waitKey(1);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lanenet_cluster_subscriber");
    ros::NodeHandle nh;

    ros::Subscriber lanenet_sub = nh.subscribe("/lane_cluster_topic",100, lanenet_callback);

    ros::spin();

    return 0;
}