#include <iomanip>
#include <iostream>
#include <fstream>
#include <math.h>
#include <map>
#include <string>
#include <vector>

#include "slam/Data.h"
#include "slam/Old_Data.h"
#include "slam/GlobalPathPoint.h"

#include "ros/package.h"
#include "ros/ros.h"
#include "ros/time.h"

using namespace std;


typedef pair<double, double> pdd;

class GlobalPathGenerator{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber sub_;
    vector<slam::GlobalPathPoint> points_;
    pdd prev_;
    pdd cur_;
    double threshold_distance_;
    std::stringstream save_path_stream;
	bool is_kcity;

    public:
    GlobalPathGenerator(){
        sub_ = nh_.subscribe("/filtered_data", 2, &GlobalPathGenerator::callback, this);
        ros::param::get("/is_kcity", is_kcity);
		points_.clear();
        prev_.first = 0.0;
        prev_.second = 0.0;
        threshold_distance_ = 0.3;
		//change the number in save_path_stream into the bag sequence
		is_kcity = true;
		if(!is_kcity) save_path_stream << ros::package::getPath("slam") << "/config/FMTC/subpath.txt";
		else save_path_stream << ros::package::getPath("slam") << "/config/KCity/path_txt/inter_G_2_right.txt";
    }
    

    void save(){
        ofstream out(save_path_stream.str());
        for(slam::GlobalPathPoint point : points_) out<<to_string(point.x)+" "+to_string(point.y)+" "+to_string(point.theta)+" "+to_string(point.flag)<<endl;
        out.close();
        printf("complete save\n");
    }

    void callback(const slam::Old_Data::ConstPtr& msg){
        cur_.first = msg->x;
        cur_.second = msg->y;

        if(sqrt((cur_.first-prev_.first)*(cur_.first-prev_.first)+(cur_.second-prev_.second)*(cur_.second-prev_.second))<threshold_distance_) return;

        slam::GlobalPathPoint point;
        point.x = cur_.first;
        point.y = cur_.second;
        point.theta = msg->theta;
	    point.flag = 3;
        prev_.first = cur_.first;
        prev_.second = cur_.second;
        points_.push_back(point);
        cout << "# of global path points: " << points_.size() << endl;

        save();
    }
    
};

int main(int argc, char **argv){
    ros::init(argc, argv, "global_path_generator");
    GlobalPathGenerator global_path_generator;
    ros::spin();
}
