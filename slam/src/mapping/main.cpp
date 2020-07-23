#include "map_cutter.h"
#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#define FMTC 1
#define KCity 2

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    MapCutter map_cutter(FMTC);

    double lat = 0;
    double lon = 0;
    double heading = 0.5;
    Mat modified_map(map_cutter.smartCut(lat, lon, heading));
}