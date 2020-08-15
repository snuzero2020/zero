#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"
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
        color_randomizer();
        in_path_stream << ros::package::getPath("slam") << "/config/capture1.jpg";
        sub_ = nh_.subscribe("/pcl_on_image", 1, &Visualizer::callback, this);
    }

    void callback(const slam::Clustermaster::ConstPtr& msg){
        //img = Mat::zeros()
        img = imread(in_path_stream.str(),IMREAD_COLOR);
        cout << "Size:" << img.rows << "<" << img.cols << endl;
        lidar_on_image = msg->clusters;
        int color_flag = 0;
        for(slam::imgCluster cluster : lidar_on_image){
            for(geometry_msgs::Point pt : cluster.points){
                if(pt.x <= img.cols && pt.y <= img.rows && pt.x >=0 && pt.y >= 0){
                    cout << pt.x << " & " << pt.y << endl;
                    //img.at<Vec3b>(pt.y, pt.x) = color_list.at(color_flag % 3);
                    circle(img, Point(pt.x,pt.y),2,color_list.at(color_flag % 3),-1);
                }
                //else cout << "Out of range" << endl;
            }
            color_flag += 1;
        }

        imshow("test", img);
        char c = waitKey(25);
        // if(c == 's'){
        //     imwrite("/home/jungwonsuhk/test.jpg",img)
        // }
        
        //imwrite("/home/jeongwoooh/vision/lidar_projected.jpg", img);
        //printf("completed to save img, img size : %d \n", img.size());
        mainclock = ros::Time::now();
    }

    void quit_if_end(){
        if((ros::Time::now() - mainclock).sec > 2){
            destroyAllWindows();
        }
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
    stringstream in_path_stream;
    Mat img;
    ros::Time mainclock;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "lidar_visualizer");
    Visualizer visualizer;
    while(ros::ok()){
        visualizer.quit_if_end();
        ros::spinOnce();
    }
    return 0;
}
