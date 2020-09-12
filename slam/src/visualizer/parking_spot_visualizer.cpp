#include <cstring>
#include <fstream>
#include <iostream>
#include <cmath>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "ros/package.h"
#include "opencv2/opencv.hpp"

#include "XYToPixel.h"


int main() {
    bool is_kcity;
    ros::param::get("/is_kcity", is_kcity);

    std::vector<std::vector<double>> coords;

    std::stringstream coord_path;
    std::stringstream map_path;

    if (!is_kcity) {
        coord_path << ros::package::getPath("slam") << "/config/FMTC/FMTC_parking_spot.txt";
        map_path << ros::package::getPath("slam") << "/config/FMTC/FMTC_map.png";

        ROS_INFO("The FMTC map path and coordinate paths loaded");
    } else {
        coord_path << ros::package::getPath("slam") << "/config/KCity/KCity_parking_spot.txt";
        map_path << ros::package::getPath("slam") << "/config/KCity/KCity.png";

        ROS_INFO("The KCIty map path and coordinate paths loaded");
    }

    std::ifstream coord_file(coord_path.str());

    cv::Mat map = cv::imread(map_path.str(), cv::IMREAD_COLOR);

    if (map.empty()) {
        ROS_ERROR("Parking Spot Visualizer: Wrong map path");
        return 0;
    } else {
        ROS_INFO("Parking Spot Visualizer: Map Loaded");
    }

    while(!coord_file.eof()) {
        char buf[100];
        std::vector<double> coord;

        coord_file.getline(buf, 100);

        char* tok1 = strtok(buf," ");

        while (tok1 != NULL) {
		    //std::cout << tok1 << ", ";
            coord.push_back(atof(tok1));

		    tok1 = strtok(NULL, " ");
	    }
        //std::cout << "\n";

        
        for (double n : coord) {
            std::cout << n << ", ";
        }
        std::cout << "\n";
        

        coords.push_back(coord);
        
    }

    coord_file.close();

    for (int count1 = 0; count1 < coords.size() - 1; count1++) {
        std::vector<double> i = coords.at(count1);

        double x = i.at(0);
        double y = i.at(1);
        double heading = i.at(2);
        double height = i.at(3);
        double width = i.at(4);

        cv::Point2d spot_point1(x + height / 2 * cos(heading) + width / 2 * sin(heading), y + height / 2 * sin(heading) - width / 2 * cos(heading));
        cv::Point2d spot_point2(x - height / 2 * cos(heading) + width / 2 * sin(heading), y - height / 2 * sin(heading) - width / 2 * cos(heading));
        cv::Point2d spot_point3(x - height / 2 * cos(heading) - width / 2 * sin(heading), y - height / 2 * sin(heading) + width / 2 * cos(heading));
        cv::Point2d spot_point4(x + height / 2 * cos(heading) - width / 2 * sin(heading), y + height / 2 * sin(heading) + width / 2 * cos(heading));

        std::cout << std::fixed\
                  << "x: " << i.at(0)\
                  << ", y: " << i.at(1)\
                  << ", heading: " << i.at(2);

        std::cout.precision(2);

        std::cout << ", height: " << i.at(3)\
                  << ", width: " << i.at(4)\
                  << std::endl;
        
        std::cout.precision(6);
        
        cv::Point2i pixel_spot_point1, pixel_spot_point2, pixel_spot_point3, pixel_spot_point4;

        XYToPixel(pixel_spot_point1.x, pixel_spot_point1.y, spot_point1.x, spot_point1.y, is_kcity);
        XYToPixel(pixel_spot_point2.x, pixel_spot_point2.y, spot_point2.x, spot_point2.y, is_kcity);
        XYToPixel(pixel_spot_point3.x, pixel_spot_point3.y, spot_point3.x, spot_point3.y, is_kcity);
        XYToPixel(pixel_spot_point4.x, pixel_spot_point4.y, spot_point4.x, spot_point4.y, is_kcity);

        cv::line(map, pixel_spot_point1, pixel_spot_point2, cv::Scalar(count1 * 20, count1 * 20, 250 - count1 * 20), 5);
        cv::line(map, pixel_spot_point2, pixel_spot_point3, cv::Scalar(count1 * 20, count1 * 20, 250 - count1 * 20), 5);
        cv::line(map, pixel_spot_point3, pixel_spot_point4, cv::Scalar(count1 * 20, count1 * 20, 250 - count1 * 20), 5);
        cv::line(map, pixel_spot_point4, pixel_spot_point1, cv::Scalar(count1 * 20, count1 * 20, 250 - count1 * 20), 5);
    }

    std::stringstream map_output_path;
    if (!is_kcity) map_output_path << ros::package::getPath("slam") << "/config/FMTC/FMTC_parking_spot_visualized_map.png";
    else map_output_path << ros::package::getPath("slam") << "/config/KCity/KCity_parking_spot_visualized_map.png";

    cv::imwrite(map_output_path.str(), map);
}