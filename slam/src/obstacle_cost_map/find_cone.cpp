#include <algorithm>
#include <iostream>
#include <math.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "geometry_msgs/Point.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "nav_msgs/OccupancyGrid.h"

#include "slam/Cluster.h"
#include "slam/Lidar.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;
using namespace cv;


class DetectCone{
    public:
    DetectCone(){
        count = 0; // CAUTION 1. You should press 'q' before program ends
        element = getStructuringElement(MORPH_RECT, Size(4,4), Point(-1,-1));
		//ptsub_ = nh_.subscribe("/2d_obstacle_clouds", 1, &DetectCone::callback, this);
        pub_ = nh_.advertise<sensor_msgs::Image>("/obstacle_map/costmap",10);
        pub_map_ = nh_.advertise<nav_msgs::OccupancyGrid>("/obstacle_cost_map",10);
        imgsub_ = nh_.subscribe("/obstacle_map/image_raw",1,&DetectCone::imagecallback,this);
        namedWindow("cone",WINDOW_AUTOSIZE);
        kernel = getKernel(31); // CAUTION 2. size should be odd! , 3cm/ 1 size
        kernel_size = kernel.rows;
    }

    ~DetectCone(){
        destroyWindow("cone");
        destroyWindow("costmap");
    }

    inline double pixel_dist(int center_x, int center_y, int dst_x, int dst_y){
        int rt = (center_x - dst_x)*(center_x -dst_x) + (center_y - dst_y)*(center_y - dst_y);
        rt = static_cast<double>(rt);  
        return sqrt(rt);
    }

    //get kernel matrix: should be handled
    Mat getKernel(int kernel_size){
        Mat rt = Mat::zeros(Size(kernel_size,kernel_size),CV_8UC1);
        int cen_x = int(rt.cols/2);
        int cen_y = int(rt.rows/2);
        for(int j=0; j<rt.rows ; j++){
            for(int i=0; i<rt.cols; i++ ){
                int distance = 100 - 5 * ceil(pixel_dist(cen_x,cen_y,i,j)); // CAUTION : change
                if (distance < 0) distance = 0;
                rt.at<uchar>(j,i) = static_cast<uchar>(distance);
            }
        }
        //cout << rt << endl;
        return rt;
    }

    void quit_if_end(){
        if((ros::Time::now() - mainclock).sec > 2){
            destroyWindow("cone");
            destroyWindow("costmap");
        }
    }

    void imagecallback(const sensor_msgs::Image::ConstPtr& msg){
        clock_t begin = clock();
        Mat labels, map, dilate_map, gray, padded_map; // map: Map subscribed, gray (gray map)
        cv_bridge::CvImagePtr map_ptr;
        
        // Getting Image message _ usable
        try{
            map_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e){
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return;
        }
        map = map_ptr->image;
        
        // Preprocessing Image
        erode(map, dilate_map, element);
        cvtColor(dilate_map, gray, COLOR_BGR2GRAY);
        int pad_size = (kernel_size-1)/2;
        copyMakeBorder(gray, padded_map,pad_size,pad_size,pad_size,pad_size,BORDER_CONSTANT,Scalar(0)); // zero-padding
        
        //CAUTION: IF LINE DETECTION REQUIRED
        // for(int y = 0; y < gray.rows; y++){
        //     for(int x = 0; x < gray.rows; x++){
        //         gray.at<uchar>(y,x) = abs(gray.at<uchar>(y,x)-255);
        //     }
        // }
        // clock_t began = clock();
        // vector<Vec4i> lines;
        // HoughLinesP(gray, lines, CV_PI/180, 20,10,20);
        // for(size_t i=0; i<lines.size(); i++){
        //     Vec4i l = lines[i];
        //     line(gray, Point(l[0],l[1]), Point(l[2],l[3]),Scalar(255,255,255),30,CV_AA);
        // }
        // clock_t final = clock();
        //ROS_INFO("elaspsed time(II) : %lf", double(final-began)/CLOCKS_PER_SEC);

        //Making Costmap : every point
        costmap = Mat::zeros(padded_map.size(),CV_8UC1);
        for(int j=0; j<padded_map.rows - kernel_size; j++){
            for(int i=0; i<padded_map.cols - kernel_size; i++){
                int center = padded_map.at<uchar>(j+pad_size, i+pad_size);
                if(center == 0){
                    for(int jj=0; jj<kernel_size; jj++){
                        for(int ii=0; ii<kernel_size; ii++){
                            int cost = kernel.at<uchar>(jj,ii);
                            int present = costmap.at<uchar>(j+jj,i+ii);
                            if (cost == 0) continue;
                            if(present < cost * (255-center)/100){
                                costmap.at<uchar>(j+jj,i+ii) = saturate_cast<uchar>(cost*(255-center)/100); //CAUTION : BLACK OR WHITE
                            }
                        }
                    }
                }
                else continue;
            }
        }
        int slice = (kernel_size - 1)/2;
        costmap_sliced = costmap(Range(slice, padded_map.rows - slice),Range(slice, padded_map.cols - slice));
        ///Rotate pi(rad)

        Point2f rotation_center(costmap_sliced.cols/2, costmap_sliced.rows/2);
        Mat rotation_matrix = getRotationMatrix2D(rotation_center, 180, 1.0);
        warpAffine(costmap_sliced, costmap_sliced, rotation_matrix, costmap_sliced.size());

        // Quit if press 'q'
        if(count == 0){
            imshow("cone", map);
            imshow("costmap",costmap_sliced);
            int key = waitKey(30);
            if(key == 'q') {
                destroyWindow("cone");
                destroyWindow("costmap");
                count += 1;
            }
        }

        // Publish Code <type> :: Image
        sensor_msgs::Image rt;
        std_msgs::Header header;
        header.seq = msg->header.seq;
        header.stamp = ros::Time::now();
        img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO8, costmap_sliced);
        img_bridge.toImageMsg(rt);
        pub_.publish(rt);

        nav_msgs::OccupancyGrid cost_map;
		cost_map.info.width = 300;
		cost_map.info.height = 300;

		for (int i = 1; i < 301; i++){
			for (int j = 1; j < 301; j++) cost_map.data.push_back(static_cast<int8_t>(costmap.at<uchar>(300-i,300-j)));
		}
						
		pub_map_.publish(cost_map);
        
        // Measure time taken at code
        clock_t end = clock();
        mainclock = ros::Time::now();
        ROS_INFO("elaspsed time : %lf", double(end-begin)/CLOCKS_PER_SEC);
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Publisher pub_map_;
    ros::Subscriber imgsub_; // Image type subscriber node
    ros::Subscriber ptsub_; // Point type subscriber node
    ros::Time mainclock; 
    int count;
    int kernel_size; // Kernel's size
    
    Mat element; // mask size_erode
    Mat kernel; // Kernel that controls the way that spread
    Mat costmap; // For costmap
    Mat costmap_sliced; // Sliced

    cv_bridge::CvImage img_bridge;
};


int main(int argc, char **argv){
    ros::init(argc, argv, "obstacle_costmap_publisher");
    DetectCone detect_cone;
    while(ros::ok()){
        detect_cone.quit_if_end();
        ros::spinOnce();
    }
    return 0;
}

// temp = map_ptr -> image;
//morphologyEx (map_ptr-> image, temp, MORPH_CLOSE, element);

//Lavacone detection

// erode(map_ptr->image, temp, element);
// cvtColor(temp, gray, COLOR_BGR2GRAY);

// for(int y = 0; y < temp.rows; y++){
//     for(int x = 0; x < temp.rows; x++){
//         gray.at<uchar>(y,x) = abs(gray.at<uchar>(y,x)-255);
//     }
// }
// vector<Vec4i> lines;
// HoughLinesP(gray, lines, 1, CV_PI/180, 20, 10, 20);
// for (size_t i=0; i<lines.size(); i++){
//     Vec4i l = lines[i];
//     line(temp, Point(l[0],l[1]), Point(l[2],l[3]), Scalar(255,255,255), 30, CV_AA);
// }
// cout << lines.size() << endl;

// Connection

// cvtColor(temp,temp,COLOR_BGR2GRAY);
// int n = connectedComponentsWithStats(temp, labels, stats, centroids);
// vector<Vec3b> colors(n+1);
// for (int i=0; i<n; i++){
//     colors[i] = Vec3b(rand() % 256, rand()%256, rand()%256);
// }
// img_color = Mat(temp.size(),CV_8UC3);
// for(int y = 0; y < temp.rows; y++){
//     for(int x = 0; x < temp.cols; x++){
//         int label = labels.at<int>(y,x);
//         img_color.at<cv::Vec3b>(y,x) = colors[label];
//     }
// }

// to show, press q to quit normally  
