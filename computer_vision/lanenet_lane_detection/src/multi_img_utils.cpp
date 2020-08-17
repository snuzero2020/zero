#include <opencv2/opencv.hpp>

#include <stdio.h>
#include <stdarg.h>

using namespace cv;
using namespace std;

void ShowManyImages(string title, int nArgs, ...) {
int size;
int i;
int m, n;
int x, y;
int w, h;

float scale;
int max;

if(nArgs <= 0) {
    printf("Number of arguments too small....\n");
    return;
}
else if(nArgs > 14) {
    printf("Number of arguments too large, can only handle maximally 12 images at a time ...\n");
    return;
}

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

Mat DispImage = Mat::zeros(Size(100 + size*w, 60 + size*h), CV_8UC1);

va_list args;
va_start(args, nArgs);

for (i = 0, m = 20, n = 20; i < nArgs; i++, m += (20 + size)) {

    Mat img = va_arg(args, Mat);

    if(img.empty()) {
        printf("Invalid arguments");
        return;
    }

    x = img.cols;
    y = img.rows;

    max = (x > y)? x: y;

    scale = (float) ( (float) max / size );

    if( i % w == 0 && m!= 20) {
        m = 20;
        n+= 20 + size;
    }

    Rect ROI(m, n, (int)( x/scale ), (int)( y/scale ));
    Mat temp; resize(img,temp, Size(ROI.width, ROI.height));
    temp.copyTo(DispImage(ROI));
}

namedWindow( title, 1 );
imshow( title, DispImage);

va_end(args);
}