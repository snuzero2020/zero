#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <vector>
#include <iostream>
#include <string.h>
#include "Eigen/Eigen"
#include "geometry_msgs/Point.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "math.h"
#include "slam/Cluster.h"
#include "slam/"

using namespace std;
using namespace cv;
using namespace Eigen;

#define PI 3.14159265

class Calibration{
    public:
    Calibration(){
        //camera internal, external parameter Measurement
        fx = 1138.58088;
        fy = 1140.27068;
        cx = 970.443632;
        cy = 486.062255;
        roll = 0.0, pitch = 19.0, yaw = 0.0;
        x = 0, y = 0.4, z = 0.1; // Unit[m] => Camera to Lidar

        //Publish, Subscribe
        pub_ = nh_.advertise<slam::Cluster>("/pcl_on_image", 10);
        sub_ = nh_.subscribe("/points", 1, &Calibration::callback, this);
    }

    void callback(const slam::Cluster::ConstPtr& msg){
        internal = getInternalMatrix(fx, fy, cx, cy);
        external = getExternalMatrix(roll, pitch, yaw, x, y, z);
        calibration_matrix = internal * external; // final_calibration_matrix
        cout << "Calibration_matrix" << endl;
        cout << calibration_matrix << endl;
        lid_x = msg->x;
        lid_y = msg->y;
        lid_z = msg->z;
        lidartoImage(
    }

    Matrix3d getInternalMatrix(double fx, double fy, double cx, double cy){
        Matrix3d internal_;
        internal_<< fx, 0.0f, cx,
                    0.0f, fy, cy,
                    0.0f, 0.0f, 1.0f;
        cout << "Internal Matrix: " << endl;
        cout << internal_ << endl;
        return internal_; 
    }

    Matrix3Xd getExternalMatrix(double roll, double pitch, double yaw, double x, double y, double z){
        Vector3d trans;
        Matrix3Xd rotd(3,3), rotx(3,3), roty(3,3), rotz(3,3);
        trans << x, y, z;
        rotd << 0.0f, -1.0f, 0.0f,
                0.0f, 0.0f, -1.0f,
                1.0f, 0.0f, 0.0f;
        rotx << 1.0f, 0.0f, 0.0f,
                0.0f, cos(PI*roll/180), -sin(PI*roll/180),
                0.0f, sin(PI*roll/180), cos(PI*roll/180);
        roty << cos(PI*pitch/180), 0.0f, -sin(PI*pitch/180),
                0.0f, 1.0f, 0.0f,
                -sin(PI*pitch/180), 0.0f, cos(PI*pitch/180);
        rotz << cos(PI*yaw/180), -sin(PI*yaw/180), 0.0f,
                sin(PI*yaw/180), cos(PI*yaw/180), 0.0f,
                0.0f, 0.0f, 1.0f;
        Matrix3Xd rot(3,3);
        rot = rotd * rotx * roty * rotz;
        rot.conservativeResize(rot.rows(), rot.cols()+1);
        rot.col(rot.cols()-1) = trans;
        cout << "External Matrix: " << endl;
        cout << rot << endl;
        return rot;
    }

    void lidartoImage(double x, double y, double z){
        Vector4d lidar_pt;
        Vector3d image_pt, camera_world;
        lidar_pt << x, y, z, 1.0;
        image_pt = calibration_matrix * lidar_pt;
        camera_world = external * lidar_pt;
        cout << image_pt(0) << "<" << image_pt(1) << "<" << image_pt(2) << endl;
        cout << camera_world(0) << "<" << camera_world(1) << "<" << camera_world(2) << endl;
        for(int i=0; i<3 ; i++){
            image_pt(i) /= image_pt(2);
        }
        cout << image_pt(0) << "," << image_pt(1) << endl;        
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    double fx, fy, cx, cy;
    double roll, pitch, yaw, x, y, z;
    Matrix3d internal;
    Matrix3Xd external;
    Matrix3Xd calibration_matrix;
    vector<int> lidar_channels;
    vector<int> lidar_clusters;
    vector<geometry_msgs::Point> lidar_points;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "calib_matrix");
    Calibration calibration_;
    ros::spin();
}