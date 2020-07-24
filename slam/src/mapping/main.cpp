#include "MapCutter.hpp"
#include <iostream>

#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"

#ifndef PLACE
#define PLACE

#define KCity 1
#define FMTC 2

#endif

using namespace std;
using namespace cv;

int main(int argc, char** argv) {
    MapCutter map_cutter(FMTC);

    double lat = 0;
    double lon = 0;
    double heading = 0.5;
    Mat modified_map(map_cutter.smartCut(lat, lon, heading));
}
