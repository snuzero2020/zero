#ifndef LANE_FIT_SLIDING_WINDOW
#define LANE_FIT_SLIDING_WINDOW

#include "opencv2/opencv.hpp"
#include <vector>

using namespace std;

int return_xbase(int *array, int size);
cv::Mat get_fits_by_sliding_window(cv::Mat img, int n_window);
int hist_sum(int *array, int size);
int return_vector_xbase(vector<int> x);
cv::Point2i get_goal_point(cv::Mat lane_mask, int size);

#endif
