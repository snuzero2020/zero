#include <iostream>
#include <math.h>
#include <vector>

#include "geometry_msgs/Point.h"

#include "slam/Yoloinfo.h"
#include "slam/Yolomaster.h"

#include "ros/package.h"
#include "ros/ros.h"
#include "ros/time.h"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;


class Lidarcapture{
    public:
    Lidarcapture(){
        capture_video();
    }
    
    int capture_video(){
        VideoCapture cap("/home/jungwonsuhk/vision/kcity_ws/my_video-6.mkv");

        if(!cap.isOpened()){
            cout << "Error opening video stream or file" << endl;
            return -1;
        }

        while(1){
            Mat frame;
            cap >> frame;
            if(frame.empty()) break;
            imshow("hello", frame);
            char c = (char)waitKey(25);
            if(c=='s'){
                imwrite("/home/jungwonsuhk/vision/capture6.jpg", frame);
                cout << "Image saved!" << endl;
            }
            if(c==27) break;
        }
        cap.release();
        destroyAllWindows();
        return 0;
    }
};


int main(int argc, char **argv){
    ros::init(argc, argv, "video_capture");
    Lidarcapture lidarcapture;
    return 0;
}
