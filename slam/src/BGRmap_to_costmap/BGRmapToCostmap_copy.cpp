#include <iostream>
#include "ros/ros.h"
#include <cstring>
#include <string>

// Opencv 3 header files
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <cmath>
#include "BGRmapToCostmap.hpp"

#include <chrono>

#include <thread>

#define FMTC 1
#define KCity 2

using namespace std;
using namespace cv;


BGRmapToCostmap::BGRmapToCostmap(Mat BGRmap) {
    this->BGRmap = BGRmap;
    
    if (BGRmap.empty()) {
        ROS_ERROR("BGRmapToCostmap: The received BGR map is empty");
    } else {
        ROS_INFO("BGRmapToCostmap: Success import the BGR map");
    }
}

void BGRmapToCostmap::setBGRmap(Mat BGRmap) {
   this->BGRmap = BGRmap;
    
    if (!BGRmap.empty()) {
        ROS_ERROR("BGRmapToCostmap: Received BGR map is empty");
        return;
    } else {
        ROS_INFO("BGRmapToCostmap: Success import the BGR map");
    }
}

Mat BGRmapToCostmap::getCostmap() {
    return costmap;
}

void BGRmapToCostmap::transform(function<double(uchar, uchar, uchar)>& weight, function<double(double)>& formula, int p_scope, double threshold, int core) {
    if (threshold < 0) {
        ROS_ERROR("BGRmapToCostmap: the threshold is under 0");
        return;
    }

    int scope;
    
    if (p_scope < 0) { // the auto-set scope
        for (int n = 0;; n++) {
            if (formula(n) < threshold) {
                scope = n;
                break;
            }
        }
    } else { // the pre-set scope
        scope = p_scope;
    }

    int cols = BGRmap.cols;
    int rows = BGRmap.rows;

    costmap.create(cols, rows, CV_8U);

    uchar* BGRmap_data = BGRmap.data;
    uchar* costmap_data = costmap.data;

    int pixels = cols * rows;
    int status = 0;
    int bbox_up, bbox_down, bbox_left, bbox_right;

    // set the all pixel of costmap to 1

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            costmap_data[i * rows + j] = 1;
        }
    }

    int elipsed_min = 0;
    int elipsed_sec = 0;
    int elipsed_msec = 0;

    int rest_min = 0;
    int rest_sec = 0;
    int rest_hour = 0;
    int rest_time = 0;

    std::chrono::system_clock::time_point start = std::chrono::system_clock::now();

    for (int i = 0; i < rows; i++) {
        if (i >= threshold) {
            thread t1()
        }

        for (int j = 0; j < cols; j++) {
            double cost_seed = weight(BGRmap_data[3 * (i * cols + j)], BGRmap_data[3 * (i * cols + j) + 1], BGRmap_data[3 * (i * cols + j) + 2]);

            if (cost_seed == 0) {
                status++;
                continue;
            }
            
            // bound the area affected by the cost seed
            // 1. rows

            if (i < scope) {
                bbox_up = 0;
            } else {
                bbox_up = i - scope;
            }

            if ((i + 1) + scope > cols) {
                bbox_down = cols - 1;
            } else {
                bbox_down = i + scope;
            }

            // 2. cols

            if (j < scope) {
                bbox_left = 0;
            } else {
                bbox_left = j - scope;
            }

            if ((j + 1) + scope > rows) {
                bbox_right = rows - 1;
            } else {
                bbox_right = j + scope;
            }

            // set the area affected by the cost seed
            Range bbox_row(bbox_left, bbox_right + 1);
            Range bbox_col(bbox_up, bbox_down + 1);

            for (int work_i = bbox_up; work_i <= bbox_down; work_i++) {
                for (int work_j = bbox_left; work_j <= bbox_right; work_j++) {
                    Point seed_point(j, i);
                    Point working_point(work_j, work_i);

                    double distance = getDistance(seed_point, working_point);

                    // add a cost

                    if (costmap_data[work_i * cols + work_j] < cost_seed * formula(distance)) {
                        costmap_data[work_i * cols + work_j] = cost_seed * formula(distance);
                    }  
                }
            }

            status++;
        }
        std::chrono::milliseconds chrono_msec  = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - start);

        elipsed_msec = chrono_msec.count();

        rest_time = static_cast<int>((rows - (i + 1)) / (i + 1) * elipsed_msec / 1000);
        rest_hour = static_cast<int>(rest_time / 3600);
        rest_min = static_cast<int>((rest_time - (rest_hour * 3600)) / 60);
        rest_sec = (rest_time - (rest_hour * 3600) - (rest_min * 60));
        
        rest_sec = rest_sec % 60;

        for (int i = 0; i < 21; i++) {
            cout << "\n" << endl;
        }

        string rest_min_str;

        if (rest_min < 10) {
            
            rest_min_str += ("0" + to_string(rest_min));
        } else {
            rest_min_str = to_string(rest_min);
        }
        
        string rest_sec_str;

        if (rest_sec < 10) {
            
            rest_sec_str += ("0" + to_string(rest_sec));
        } else {
            rest_sec_str = to_string(rest_sec);
        }

        ROS_INFO("%lf%% completed (%d/%d)", static_cast<double>(status) / static_cast<double>(pixels) * 100, status, pixels);
        ROS_INFO("complete after %d:%s:%s", rest_hour, rest_min_str.c_str(), rest_sec_str.c_str());
    }

    // if a cost is over 100, the cost must cut to 100

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (costmap_data[i * rows + j] > 100) {costmap_data[i * rows + j] = 100;}
        }
    }
}

double BGRmapToCostmap::getDistance(Point point1, Point point2) {
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));
}