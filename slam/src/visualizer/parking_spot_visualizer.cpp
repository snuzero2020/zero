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
    ros::param::get("/is_kcity", is_kcity); // get whether the target is for K-City or FMTC (변환 대상이 K-City인지 FMTC인지를 가져옴)
    std::vector<std::vector<double>> coords; // list for coordinates (좌표 여러개를 저장할 리스트)

    std::stringstream coord_path; // path of list for coordinates (변환할 좌표 목록이 저장된 txt 파일)
    std::stringstream map_path; // path of map (배경 지도 파일)

    if (!is_kcity) { // FMTC
        coord_path << ros::package::getPath("slam") << "/config/FMTC/FMTC_parking_spot.txt"; // enter the path of the list
        map_path << ros::package::getPath("slam") << "/config/FMTC/FMTC_map.png"; // enter the path of the map

        ROS_INFO("The FMTC map path and coordinate paths loaded");
    } else { // K-City
        coord_path << ros::package::getPath("slam") << "/config/KCity/KCity_parking_spot.txt"; // enter the path of the list
        map_path << ros::package::getPath("slam") << "/config/KCity/KCity.png"; // enter the path of the map

        ROS_INFO("The KCIty map path and coordinate paths loaded");
    }

    std::ifstream coord_file(coord_path.str()); // list for coordinates (좌표 목록 txt 파일)

    cv::Mat map = cv::imread(map_path.str(), cv::IMREAD_COLOR); // map file (지도 파일 2차원 리스트)

    if (map.empty()) { // the map file do not exist (지도 파일 X 또는 잘못된 경로)
        ROS_ERROR("Parking Spot Visualizer: Wrong map path");
        return 0;
    } else {
        ROS_INFO("Parking Spot Visualizer: Map Loaded");
    }

    while(!coord_file.eof()) { // read coordinate file fully (좌표 파일을 끝까지 읽음)
        char buf[100]; // string buffer (문자열 버퍼)
        std::vector<double> coord; // var list for a coordinate (단일 좌표를 담을 변수 목록)

        coord_file.getline(buf, 100); // read one line (1줄 읽기)

        char* tok1 = strtok(buf," "); // split via space & save the former (공백을 기준으로 앞쪽 문자열을 저장)

        while (tok1 != NULL) { // entirely split the string (문자열을 완전히 쪼갬)
		    //std::cout << tok1 << ", ";
            coord.push_back(atof(tok1));

		    tok1 = strtok(NULL, " ");
	    }
        //std::cout << "\n";

        
        for (double n : coord) { // print the splited coordinate string (쪼갠 좌표를 하나씩 출력)
            std::cout << n << ", ";
        }
        std::cout << "\n";
        

        coords.push_back(coord); // save the splited coordiate to 'coords' var (쪼갠 좌표 목록을 coords에 저장)
    }

    coord_file.close();

    for (int count1 = 0; count1 < coords.size() - 1; count1++) {
        std::vector<double> i = coords.at(count1); // 'i'th lat-lon coordinate (i번째 좌표)

        double x = i.at(0);
        double y = i.at(1);
        double heading = i.at(2);
        double height = i.at(3);
        double width = i.at(4);

        // calculate coordinates of the each corner (각 꼭짓점의 좌표를 계산)
        cv::Point2d spot_point1(x + height / 2 * cos(heading) + width / 2 * sin(heading), y + height / 2 * sin(heading) - width / 2 * cos(heading));
        cv::Point2d spot_point2(x - height / 2 * cos(heading) + width / 2 * sin(heading), y - height / 2 * sin(heading) - width / 2 * cos(heading));
        cv::Point2d spot_point3(x - height / 2 * cos(heading) - width / 2 * sin(heading), y - height / 2 * sin(heading) + width / 2 * cos(heading));
        cv::Point2d spot_point4(x + height / 2 * cos(heading) - width / 2 * sin(heading), y + height / 2 * sin(heading) + width / 2 * cos(heading));

        // print xy coordinates of the each corner (각 꼭짓점의 xy 좌표를 출력)
        std::cout << std::fixed\
                  << "x: " << i.at(0)\
                  << ", y: " << i.at(1)\
                  << ", heading: " << i.at(2);

        std::cout.precision(2);

        std::cout << ", height: " << i.at(3)\
                  << ", width: " << i.at(4)\
                  << std::endl;
        
        std::cout.precision(6);
        
        cv::Point2i pixel_spot_point1, pixel_spot_point2, pixel_spot_point3, pixel_spot_point4; // pixel coordinate of the each corner (각 꼭짓점의 픽셀 좌표)

        // transform the xy coordinates to the pixel coordinates (xy 좌표를 픽셀 좌표로 변환)
        XYToPixel(pixel_spot_point1.x, pixel_spot_point1.y, spot_point1.x, spot_point1.y, is_kcity);
        XYToPixel(pixel_spot_point2.x, pixel_spot_point2.y, spot_point2.x, spot_point2.y, is_kcity);
        XYToPixel(pixel_spot_point3.x, pixel_spot_point3.y, spot_point3.x, spot_point3.y, is_kcity);
        XYToPixel(pixel_spot_point4.x, pixel_spot_point4.y, spot_point4.x, spot_point4.y, is_kcity);

        // draw lines between the four edges (지도 그림 상에서 네 꼭짓점을 선으로 이음)
        cv::line(map, pixel_spot_point1, pixel_spot_point2, cv::Scalar(count1 * 20, count1 * 20, 250 - count1 * 20), 5);
        cv::line(map, pixel_spot_point2, pixel_spot_point3, cv::Scalar(count1 * 20, count1 * 20, 250 - count1 * 20), 5);
        cv::line(map, pixel_spot_point3, pixel_spot_point4, cv::Scalar(count1 * 20, count1 * 20, 250 - count1 * 20), 5);
        cv::line(map, pixel_spot_point4, pixel_spot_point1, cv::Scalar(count1 * 20, count1 * 20, 250 - count1 * 20), 5);
    }

    // print the drawed map (덧그린 지도 출력)
    std::stringstream map_output_path;
    if (!is_kcity) map_output_path << ros::package::getPath("slam") << "/config/FMTC/FMTC_parking_spot_visualized_map.png";
    else map_output_path << ros::package::getPath("slam") << "/config/KCity/KCity_parking_spot_visualized_map.png";

    cv::imwrite(map_output_path.str(), map);
}
