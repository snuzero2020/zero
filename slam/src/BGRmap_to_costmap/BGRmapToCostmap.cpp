#include <iostream>
#include "ros/ros.h"
#include <cstring>
#include <string>

// Opencv 3 header files
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"

#include <cmath>
#include "BGRmapToCostmap.h"

#include <chrono>

#include <thread>

#define FMTC 1
#define KCity 2

using namespace std;
using namespace cv;

Mat BGRmapToCostmap::BGRmap = Mat::zeros(1, 1, CV_8UC3);
Mat BGRmapToCostmap::costmap = Mat::ones(1, 1, CV_8U);

int BGRmapToCostmap::cols = 0;
int BGRmapToCostmap::rows = 0;
int BGRmapToCostmap::pixels = 0;

int BGRmapToCostmap::scope = 0;
int BGRmapToCostmap::threshold = 0;

uchar* BGRmapToCostmap::BGRmap_data = BGRmap.data;
uchar* BGRmapToCostmap::costmap_data = costmap.data;

function<double(uchar, uchar, uchar)> BGRmapToCostmap::weight = [](uchar a, uchar b, uchar c){return 1.0;};
function<double(double)> BGRmapToCostmap::formula = [](double a){return 1.0;};
std::chrono::system_clock::time_point BGRmapToCostmap::timeStart = std::chrono::system_clock::now();

int BGRmapToCostmap::elipsed_min = 0;
int BGRmapToCostmap::elipsed_sec = 0;
int BGRmapToCostmap::elipsed_msec = 0;

int BGRmapToCostmap::rest_min = 0;
float BGRmapToCostmap::rest_sec = 0;
int BGRmapToCostmap::rest_hour = 0;

vector<int> BGRmapToCostmap::thread_ids(1);

double BGRmapToCostmap::rest_time = 0;

int BGRmapToCostmap::bbox_up = 0;
int BGRmapToCostmap::bbox_down = 0;
int BGRmapToCostmap::bbox_left = 0;
int BGRmapToCostmap::bbox_right = 0;

BGRmapToCostmap::BGRmapToCostmap(Mat p_BGRmap) {
    BGRmap = p_BGRmap;
    
    if (BGRmap.empty()) {
        ROS_ERROR("BGRmapToCostmap: The received BGR map is empty");
    } else {
        ROS_INFO("BGRmapToCostmap: Success import the BGR map");
    }
}

void BGRmapToCostmap::setBGRmap(Mat p_BGRmap) {
   BGRmap = p_BGRmap;
    
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

void BGRmapToCostmap::transform(function<double(uchar, uchar, uchar)>& p_weight, function<double(double)>& p_formula, int p_scope, double p_threshold, int cores) {
    threshold = p_threshold;
    weight = p_weight;
    formula = p_formula;
    
    if (threshold < 0) {
        ROS_ERROR("BGRmapToCostmap: the threshold is under 0");
        return;
    }
    
    if (p_scope < 0) { // the auto-set scope
        for (int n = 0;; n++) {
            if (formula(n) * 100 < threshold) {
                scope = n;
                break;
            }
        }
    } else { // the pre-set scope
        scope = p_scope;
    }

    cols = BGRmap.cols;
    rows = BGRmap.rows;
    pixels = cols * rows;

    costmap.create(cols, rows, CV_8U);

    BGRmap_data = BGRmap.data;
    costmap_data = costmap.data;

    // set the all pixel of costmap to 1

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            costmap_data[i * rows + j] = 1;
        }
    }

    timeStart = std::chrono::system_clock::now();

    thread_ids.resize(cores);

    vector<thread> threads;
    threads.push_back(thread(calculateCost, 0, (rows / cores) - 1, 0));

    
    for (int n = 0; n < cores - 2; n++) {
        threads.push_back(thread(calculateCost, (rows / cores) * (n + 1), (rows / cores) * (n + 2) - 1, n + 1));
    }

    threads.push_back(thread(calculateCost, (rows / cores) * (cores - 1), rows - 1, cores - 1));
    
    for (int n = 0; n < threads.size(); n++) {
        threads[n].join();
    }

    std::chrono::milliseconds elipsed_time_sum  = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - timeStart);

    double elipsed_time_sum_double = elipsed_time_sum.count() / 1000;

    int elipsed_hour = static_cast<int>(elipsed_time_sum_double / 3600);
    int elipsed_min = static_cast<int>((elipsed_time_sum_double - (elipsed_time_sum_double * 3600)) / 60);
    int elipsed_sec = (elipsed_time_sum_double - (elipsed_hour * 3600) - (elipsed_min * 60));

    for (int i = 0; i < 21; i++) {
        cout << "\n" << endl;
    }

    string elipsed_min_str;

    if (elipsed_min < 10) {
        
        elipsed_min_str += ("0" + to_string(elipsed_min));
    } else {
        elipsed_min_str = to_string(elipsed_min);
    }
    
    string elipsed_sec_str;

    if (elipsed_sec < 10) {
        
        elipsed_sec_str += ("0" + to_string(elipsed_sec));
    } else {
        elipsed_sec_str = to_string(elipsed_sec);
    }

    ROS_INFO("Elipsed time: %d:%s:%s", elipsed_hour, elipsed_min_str.c_str(), elipsed_sec_str.c_str());
}

double BGRmapToCostmap::getDistance(Point point1, Point point2) {
    return sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2));
}

int BGRmapToCostmap::getDistanceInt(Point point1, Point point2) {
    return static_cast<int>(sqrt(pow(point2.x - point1.x, 2) + pow(point2.y - point1.y, 2)));
}

void BGRmapToCostmap::calculateCost(int row_start, int row_end, int id) {
    int proceed_rows = 0;

    for (int i = row_start; i < row_end; i++) {
        for (int j = 0; j < cols; j++) {
            double cost_seed = weight(BGRmap_data[3 * (i * cols + j)], BGRmap_data[3 * (i * cols + j) + 1], BGRmap_data[3 * (i * cols + j) + 2]);

            /*if (id == 11 ) {
                cout << +BGRmap_data[3 * (i * cols + j)] << endl;
                cout << +BGRmap_data[3 * (i * cols + j) + 1] << endl;
                cout << +BGRmap_data[3 * (i * cols + j) + 2] << endl;
                cout << weight(0,0,0) << endl;
                cout << cost_seed << "\n" << endl;
            }*/
            

            if (cost_seed < threshold) {
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

            for (int work_i = bbox_up; work_i <= bbox_down; work_i++) {
                for (int work_j = bbox_left; work_j <= bbox_right; work_j++) {
                    Point seed_point(j, i);
                    Point working_point(work_j, work_i);

                    int distance = getDistanceInt(seed_point, working_point);
                    //double distance = getDistance(seed_point, working_point);

                    // add a cost

                    if (costmap_data[work_i * cols + work_j] < cost_seed * formula(distance)) {
                        costmap_data[work_i * cols + work_j] = cost_seed * formula(distance);
                    }  
                }
            }
        }
        thread_ids[id]++;

        std::chrono::milliseconds chrono_msec  = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now() - timeStart);

        proceed_rows = 0;
        
        for (int n = 0; n < thread_ids.size(); n++) {
            proceed_rows += thread_ids[n];
        }

        elipsed_msec = chrono_msec.count();

        rest_time = ((rows - proceed_rows) / (proceed_rows + 1.0)) * elipsed_msec / 1000.0;
        rest_hour = static_cast<int>(rest_time / 3600);
        rest_min = static_cast<int>((rest_time - (rest_hour * 3600)) / 60);
        rest_sec = rest_time - (rest_hour * 3600) - (rest_min * 60);

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

        ROS_INFO("%lf%% completed (%d/%d)", static_cast<double>(proceed_rows) / static_cast<double>(rows) * 100, proceed_rows, rows);
        ROS_INFO("complete after %d:%s:%s", rest_hour, rest_min_str.c_str(), rest_sec_str.c_str());
        
    }

    // if a cost is over 100, the cost must cut to 100

    for (int i = 0; i < rows; i++) {
        for (int j = 0; j < cols; j++) {
            if (costmap_data[i * rows + j] > 100) {costmap_data[i * rows + j] = 100;}
        }
    }
}
