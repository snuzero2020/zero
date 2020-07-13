#ifndef LANE_CLASS
#define LANE_CLASS

#include "opencv2/opencv.hpp"
#include <vector>

using namespace std;

class Lane 
{
    private:
        // 0 : white, 1 : yellow
        vector<int> pixel_x;
        vector<int> pixel_y;

    public:
        double coeff[3];
        int color;
        Lane();
        void set_color(int color);
        void input_xy(int x, int y);
        int get_color();
        void get_coeff();
        void half_lane_print(int size, int side);
};

#endif