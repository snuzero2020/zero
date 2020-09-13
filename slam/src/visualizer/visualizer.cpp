#include <iostream>
#include <stdarg.h>

#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.h"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"

using namespace cv;
using namespace std;

void ShowManyImages(string title, int nArgs, ...) {
	int size;
	int i;
	int m, n;
	int x, y;

	// w - Maximum number of images in a row
	// h - Maximum number of images in a column
	int w, h;
	
	// scale - How much we have to resize the image
	float scale;
	int max;
	
	// If the number of arguments is lesser than 0 or greater than 12
	// return without displaying
	if(nArgs <= 0) {
	    printf("Number of arguments too small....\n");
	    return;
	}
	else if(nArgs > 14) {
	    printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
	    return;
	}
	// Determine the size of the image,
	// and the number of rows/cols
	// from number of arguments
	else if (nArgs == 1) {
	    w = h = 1;
	    size = 300;
	}
	else if (nArgs == 2) {
	    w = 2; h = 1;
	    size = 300;
	}
	else if (nArgs == 3 || nArgs == 4) {
	    w = 2; h = 2;
	    size = 300;
	}
	else if (nArgs == 5 || nArgs == 6) {
	    w = 3; h = 2;
	    size = 200;
	}
	else if (nArgs == 7 || nArgs == 8) {
	    w = 4; h = 2;
	    size = 200;
	}
	else {
	    w = 4; h = 3;
	    size = 150;
	}
	
	// Create a new 3 channel image
	Mat DispImage = Mat::zeros(Size(100 + size*w, 60 + size*h), CV_8UC3);
	
	// Used to get the arguments passed
	va_list args;
	va_start(args, nArgs);
	
	// Loop for nArgs number of arguments
	for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {
	    // Get the Pointer to the IplImage
	    Mat img = va_arg(args, Mat);
	
	    // Check whether it is NULL or not
	    // If it is NULL, release the image, and return
	    if(img.empty()) {
	        printf("Invalid arguments");
	        return;
	    }
	
	    // Find the width and height of the image
	    x = img.cols;
	    y = img.rows;
	
	    // Find whether height or width is greater in order to resize the image
	    max = (x > y)? x: y;
	
	    // Find the scaling factor to resize the image
	    scale = (float) ( (float) max / size );
	
	    // Used to Align the images
	    if( i % w == 0 && m!= 20) {
	        m = 20;
	        n+= 20 + size;
	    }
	
	    // Set the image ROI to display the current image
	    // Resize the input image and copy the it to the Single Big Image
	    Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
	    Mat temp; resize(img,temp, Size(ROI.width, ROI.height));
	    temp.copyTo(DispImage(ROI));
	}
	
	// Create a new window, and show the Single Big Image
	namedWindow( title, 1 );
	imshow( title, DispImage);
	waitKey();
	
	// End the number of arguments
	va_end(args);
}

class FinalVisual{
	private:
		ros::NodeHandle nh;
		ros::Publisher pub;
		ros::Subscriber minimap_sub;
		ros::Subscriber vision_sub;
		ros::Subscriber obstacle_costmap_sub;
		ros::Subscriber local_costmap_sub;
		
	public:
		int minimap_size{1000}, vision_size{600}, obstacle_costmap_size{300}, local_costmap_size{300};
		Mat minimap = Mat(minimap_size, minimap_size, CV_8UC3, cv::Scalar(0,0,0));
		Mat vision = Mat(vision_size, vision_size, CV_8UC3, cv::Scalar(0,0,0));
		Mat obstacle_costmap = Mat(obstacle_costmap_size, obstacle_costmap_size, CV_8UC3, cv::Scalar(0,0,0));
		Mat local_costmap = Mat(local_costmap_size, local_costmap_size, CV_8UC3, cv::Scalar(0,0,0));

		FinalVisual(){	
			minimap_sub = nh.subscribe("/minimap", 2, &FinalVisual::minimap_callback, this);
			//vision_sub = nh.subscribe("/vision_image", 2, &FinalVisual::vision_callback, this);
			obstacle_costmap_sub = nh.subscribe("/obstaclemap/decaying_costmap", 2, &FinalVisual::obstacle_costmap_callback, this);
			local_costmap_sub = nh.subscribe("/local_costmap", 2, &FinalVisual::local_costmap_callback, this);
		}

		void minimap_callback(const sensor_msgs::Image::ConstPtr& msg){
			cv_bridge::CvImagePtr minimap_ptr;
			minimap_ptr = cv_bridge::toCvCopy(msg, "bgr8");
			minimap = minimap_ptr->image;
			ShowManyImages("visualizers", 4, minimap, vision, obstacle_costmap, local_costmap);
		}
		
		/*
		void vision_callback(const sensor_msgs::Image::ConstPtr& msg){
			cv_bridge::CvImagePtr vision_ptr;
			vision_ptr = cv_bridge::toCvCopy(msg, "bgr8");
			vision = vision_ptr->image;
			ShowManyImages("visualizers", 4, minimap, vision, obstacle_costmap, local_costmap);
		}
		*/	
		void obstacle_costmap_callback(const sensor_msgs::Image::ConstPtr& msg){
			cv_bridge::CvImagePtr obstacle_costmap_ptr;
			obstacle_costmap_ptr = cv_bridge::toCvCopy(msg, "bgr8");
			obstacle_costmap = obstacle_costmap_ptr->image;
			ShowManyImages("visualizers", 4, minimap, vision, obstacle_costmap, local_costmap);
		}

		void local_costmap_callback(const sensor_msgs::Image::ConstPtr& msg){
			cv_bridge::CvImagePtr local_costmap_ptr;
			local_costmap_ptr = cv_bridge::toCvCopy(msg, "bgr8");
			local_costmap = local_costmap_ptr->image;
			ShowManyImages("visualizers", 4, minimap, vision, obstacle_costmap, local_costmap);
		}
};

int main(int argc, char **argv){
	ros::init(argc, argv, "final_visualizer");
	FinalVisual finalvisual;
	ros::spin();
	return 0;
}
