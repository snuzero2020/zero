#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/optflow.hpp>
#include <iostream>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <ctime>


using namespace cv;
using namespace std;

#define CENTER_X_PER_IMG_COLS 0.5
#define CENTER_Y_PER_IMG_ROWS 0.5
#define POS_STEP 1
#define ANG_STEP 0.32
#define STEP_N_2 3

void imshowall(vector<Mat> imgs);
//imgs의 원소 img들을 가로로 순서대로 붙여서 imshow

Mat warpEuclid(Mat img, int dx, int dy, double dth, bool inverse = false); 
//img를 x방향으로 dx, y방향으로 dy, th방향으로 dth 강체이동한 새 이미지 리턴

void recurMatch(int& dx, int& dy, double& dth, Mat img1, Mat img2, int iter){
    if(iter==0){
        return;
    }
    Mat small_img1;
    Mat small_img2;
    Mat temp_img1;
    Mat temp_img2;
    resize(img1, small_img1, img1.size()/2, 0, 0, INTER_AREA);
    resize(img2, small_img2, img2.size()/2, 0, 0, INTER_AREA);
    recurMatch(dx,dy,dth,small_img1,small_img2,iter-1);
    dx *= 2;
    dy *= 2;
    Rect lane_rect = Rect((img1.cols-img2.cols)/2, img1.rows/2-img2.rows, img2.cols, img2.rows); 
    int min_cross = 200*200;
    double min_dth = dth;
    int ang_step_n_2 = (iter<=1)?(M_PI_2/ANG_STEP):(STEP_N_2);
    for(int i = -ang_step_n_2; i<=ang_step_n_2; i++){
        temp_img1=warpEuclid(img1,dx,dy,dth+i*ANG_STEP/(1<<iter),true);
        int cross = norm(temp_img1(lane_rect),img2);
        if(cross<min_cross){
            min_cross = cross;
            min_dth = dth+i*ANG_STEP/(1<<iter);
        }
    }
    dth=min_dth;
    min_cross = 200*200;
    double min_dx = dx;
    double min_dy = dy;
    for(int i = -STEP_N_2; i<=STEP_N_2; i++){
        for(int j = -STEP_N_2; j<=STEP_N_2; j++){
            temp_img1=warpEuclid(img1,dx+i*POS_STEP,dy+j*POS_STEP,dth,true);
            int cross = norm(temp_img1(lane_rect),img2);
            if(cross<min_cross){
                min_cross = cross;
                min_dx = dx+i*POS_STEP;
                min_dy = dy+j*POS_STEP;
            }
        }
    }
    dx = min_dx;
    dy = min_dy;

    temp_img1=warpEuclid(img1,dx,dy,dth,true);    
    //imshowall({img1(lane_rect), img2, temp_img1(lane_rect), temp_img1(lane_rect)-img2+128.});
    //ROS_INFO("%d %d %lf",dx,dy,dth);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lane_matcher");
    ros::NodeHandle n;
    std::stringstream path_stream1;
	std::stringstream path_stream2;
	path_stream1 << ros::package::getPath("slam") << "/config/match1.png";
    path_stream2 << ros::package::getPath("slam") << "/config/match2.png";
    cv::Mat img1;
    cv::Mat img2;

    img1 = cv::imread(path_stream1.str(),cv::IMREAD_GRAYSCALE);
    if(img1.empty()){
        puts("no match1");
        return -1;
    }
    int realx = 19;  //////////////////////////////////////////////////////
    int realy = 19;    ////////////////////////////////////////////////////
    double realth = 0.51;  ////////////////////////////////////////////////////
    img1=warpEuclid(img1,realx,realy,realth,false);

    img2= cv::imread(path_stream2.str(),cv::IMREAD_GRAYSCALE);
    if(img2.empty()){
        puts("no match2");
        return -1;
    }

    Rect lane_rect = Rect((img1.cols-img2.cols)/2, img1.rows/2-img2.rows, img2.cols, img2.rows); 
    //imshowall({img1(lane_rect),img2});
    
    ros::Time t0 = ros::Time::now();

    int estx = 0; 
    int esty = 0;
    double estth = 0;
    recurMatch(estx, esty, estth, img1, img2, 5);
    ROS_INFO("%d %d %lf",estx,esty,estth);
    ROS_INFO("%lf", (ros::Time::now()-t0).toSec());
    Mat resimg = warpEuclid(img1,estx,esty,estth,true);
    imshowall({img1(lane_rect), img2, resimg(lane_rect), resimg(lane_rect)-img2+128.});
    return 0;
}


void imshowall(vector<Mat> imgs){
    Mat imgH;
    imgs[0].convertTo(imgH,CV_8U);
    for(int i=1;i<imgs.size();i++){
        Mat temp;
        imgs[i].convertTo(temp,CV_8U);
        hconcat(imgH,temp, imgH);
    }
    cv::imshow("Result Image", imgH);
    cv::waitKey(1000000);
}

Mat warpEuclid(Mat img, int x, int y, double th, bool inverse){
    double x0=img.cols*CENTER_X_PER_IMG_COLS;
    double y0=img.rows*CENTER_Y_PER_IMG_ROWS;
    Point2f srcPoint[] = {Point2f(x0, y0), Point2f(x0+1., y0), Point2f(x0, y0+1.)};
    Point2f dstPoint[] = {
        Point2f(x0 +x, y0 +y), 
        Point2f(x0 +x +cos(th), y0 +y +sin(th)), 
        Point2f(x0 +x -sin(th), y0 +y +cos(th))
    };
    Mat trans;
    if(inverse==false){
        trans = getAffineTransform(srcPoint,dstPoint);
    }else{
        trans = getAffineTransform(dstPoint,srcPoint);
    }
    Mat res;
    warpAffine(img,res,trans,img.size());
    return res;
}
