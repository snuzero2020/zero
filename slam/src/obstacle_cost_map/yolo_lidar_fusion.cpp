#include <algorithm>
#include <iostream>
#include <math.h>
#include <vector>

#include "geometry_msgs/Point.h"

#include "slam/Clustermaster.h"
#include "slam/imgCluster.h"
#include "slam/Yoloinfo.h"
#include "slam/Yolomaster.h"

#include "ros/ros.h"
#include "ros/time.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


class YoloFusion{
    public:
    YoloFusion(){
        pcl_sub_ = nh_.subscribe("/pcl_on_image",1, &YoloFusion::lidarcallback, this);
        yolo_sub_ = nh_.subscribe("/yolo_info",1, &YoloFusion::yolocallback, this);
    }

    void lidarcallback(const slam::Clustermaster::ConstPtr& msg){
        pcl_master = msg -> clusters;
        cout << pcl_master.size() << endl;
        for(int i = 0; i < pcl_master.size(); i++){
            pcl_master[i].points.clear();
        }
        pcl_master.clear();
        pcl_master = msg -> clusters;
    }
    
    //Need to be synchronized
    void yolocallback(const slam::Yolomaster::ConstPtr& msg){
        vector_initializer();
        vector<slam::imgCluster> pcl = pcl_master;
        yolo_master = msg->yolomaster;
        //Box iteration
        for(slam::Yoloinfo box : yolo_master){
            int cluster_num = 0; // Cluster Number Initialization
            vector<double> box_prob; // 2D Array of Probabilities:: Row(Label)/ Column(Cluster)
            label_candidate.push_back(box.label); // Record Label Used
            cout << "Box # " << box.label << " pushed" << endl;
            //Cluster Iteration
            for(slam::imgCluster cluster : pcl){
                int yolo_true = 0; // Count the # of the cluster
                // Point Iteration
                for(geometry_msgs::Point point : cluster.points){
                    if(isIncluded(box, point) == true) 
                        yolo_true += 1;
                }
                cout << "Cluster num: " << cluster_num << endl;
                cout << "In yolo box: " << yolo_true << endl;
                cout << "Total point: " << cluster.count << endl;
                box_prob.push_back(1.0*yolo_true/cluster.count);
                cluster_num ++;
            }
            probabilities.push_back(box_prob);
        }

        for(auto row : probabilities){
            int max_index = max_element(row.begin(), row.end())- row.begin();
            cluster_candidate.push_back(max_index);
        }

        cout << "Cluster candidate size: " << cluster_candidate.size() << endl;
        cout << "Label candidate size: " << label_candidate.size() << endl;
        //For Validation
        for(int i = 0; i < cluster_candidate.size(); i++){
            cout << label_candidate[i] << "might be " << cluster_candidate[i] << "th cluster in " << 100 * probabilities[i].at(cluster_candidate[i]) << "% possibility" << endl;
        }

        sort(cluster_candidate.begin(), cluster_candidate.end());
    }

    bool isIncluded(slam::Yoloinfo box_, geometry_msgs::Point point_){
        int w =  box_.width;
        int h = box_.height;
        geometry_msgs::Point pt = box_.points;
        // cout << "box: " << pt.x << "," << pt.y << endl;
        // cout << "width: " << w << "height: " << h << endl;
        // cout << "POint:" << point_.x << "," << point_.y << endl;
        if((point_.x <= pt.x + w/2) && (point_.x >= pt.x - w/2) && (point_.y <= pt.y + h/2) && (point_.y >= pt.y - h/2)) return true;
        else return false;
    }

    //Initialize all the vectors
    void vector_initializer(){
        yolo_master.clear();
        label_candidate.clear();
        cluster_candidate.clear();
        for(int i=0; i< probabilities.size(); i++){
            probabilities[i].clear();
        }
        probabilities.clear();
    }

    private:
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber yolo_sub_;
    ros::Subscriber pcl_sub_;

    // NEW, NEAT, SEXY, OH MY GOD!!
    vector<slam::Yoloinfo> yolo_master; // Array of YoloInfo[]  :: Point points, int width, int height, int label
    vector<slam::imgCluster> pcl_master; // Array of imgCluster[] :: Point[] points, int count
    vector<vector<double>> probabilities; // Cluster - Label Matching vectors
    vector<int> label_candidate;
    vector<int> cluster_candidate;
};

int main(int argc, char **argv){
    ros::init(argc, argv, "yolo_lidar");
    YoloFusion yolofusion;
    ros::spin();
}
