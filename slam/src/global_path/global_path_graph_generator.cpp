#include "ros/ros.h"
#include "ros/time.h"
#include "ros/package.h"
#include "slam/Imu.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Quaternion.h"
#include "slam/GlobalPathPoint.h"
#include "UnixtimeToSec.h"
#include "nav_msgs/Path.h"
#include "slam/Data.h"
#include <iostream>
#include <iomanip>
#include <string>
#include <map>
#include <math.h>
#include <vector>
#include <cmath>
#include<iostream>
#include<fstream>
#include<sstream>
#include<string>
#include<nav_msgs/OccupancyGrid.h>

using namespace std;

double dist(slam::GlobalPathPoint a, slam::GlobalPathPoint b){
    return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
}

int main(int argc, char** argv){
    ros::init(argc, argv, "global_path_graph_generator");
    
    stringstream input_stream;
    stringstream output_stream;
    vector<slam::GlobalPathPoint> points;
    vector<vector<int>> edges;
    const char delimiter = ' ';
    double distance_tolerance = 10.5;
    int count = 0;

    input_stream << ros::package::getPath("slam") << "/src/global_path/global_path.txt";
    output_stream << ros::package::getPath("slam") << "/src/global_path/global_path_graph.txt";

    // load global path points
    string in_line;
    ifstream in(input_stream.str());
    while(getline(in, in_line)){
        stringstream ss(in_line);
        string token;
        vector<string> result;
        while(getline(ss, token, delimiter)){
            result.push_back(token);
        }
        slam::GlobalPathPoint point;
        point.x = stod(result.at(0));
        point.y = stod(result.at(1));
        point.theta = stod(result.at(2));
        point.flag = stod(result.at(3));
        points.push_back(point);
        count ++;
        printf("\rloading global path points : %5d", count);    
    }
    printf("\ncomplete to load points, # of global path points : %5d\n", count);

    // build edge adjacent matrix
    for(int i=0; i<count;i++){
        vector<int> edge;
        for(int j = 0; j<count; j++){
            if(i==j) continue;
            if(dist(points.at(i),points.at(j))<distance_tolerance){
                edge.push_back(j);
            }
        }
        edges.push_back(edge);
        printf("\rgenerating edges : %5d/%5d", i+1,count);
    }
    printf("\ncompleted to generate edges\n");
    // save global path graph info
    ofstream out(output_stream.str());
    out << to_string(count) << endl;
    for(slam::GlobalPathPoint point : points) out<<to_string(point.x)+" "+to_string(point.y)+" "+to_string(point.theta)+" "+to_string(point.flag)<<endl;
    for(vector<int> edge : edges){
        for(int index : edge) out << to_string(index)+" ";
        out << endl;
    }
    out.close();
    printf("completed to save global path graph\n");
}