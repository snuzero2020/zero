#include "opencv2/opencv.hpp"
#include "lane_class.h"
#include "lane_math.h"
#include <vector>

using namespace std;

Lane::Lane()
{
    for(int i=0; i<4; i++)
    {
        coeff[i] = 0;
    }
    color = -1;
}

void Lane::set_color(int color)
{
    this->color = color;
}

void Lane::input_xy(int x, int y)
{
    pixel_x.push_back(x);
    pixel_y.push_back(y);
}

int Lane::get_color()
{
    return color;
}

void Lane::get_coeff()
{
    double A[4][4];
    double B[4];
    int size = pixel_x.size();

    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            A[i][j] = 0;
        }
    }
    
    A[0][0] = static_cast<double>(size);
    for(int i=0; i<size; i++)
    {
        A[0][1] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 1));
        A[1][0] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 1));

        A[0][2] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 2));
        A[1][1] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 2));
        A[2][0] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 2));

        A[0][3] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 3));
        A[1][2] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 3));
        A[2][1] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 3));
        A[3][0] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 3));
    
        A[1][3] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 4));
        A[2][2] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 4));
        A[3][1] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 4));

        A[2][3] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 5));
        A[3][2] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 5));

        A[3][3] += static_cast<double>(x_and_y_pow(0, 0, pixel_y[i], 6));

        B[0] += static_cast<double>(x_and_y_pow(pixel_x[i], 1, 0, 0));
        B[1] += static_cast<double>(x_and_y_pow(pixel_x[i], 1, pixel_y[i], 1));
        B[2] += static_cast<double>(x_and_y_pow(pixel_x[i], 1, pixel_y[i], 2));
        B[3] += static_cast<double>(x_and_y_pow(pixel_x[i], 1, pixel_y[i], 3));
    }

    double inverse_A[4][4];

    inverse_A[0][0] = A[1][1]*A[2][2]*A[3][3]+A[1][2]*A[2][3]*A[3][1]+A[1][3]*A[2][1]*A[3][2]-A[1][3]*A[2][2]*A[3][1]-A[1][2]*A[2][1]*A[3][3]-A[1][1]*A[2][3]*A[3][2];
    inverse_A[1][0] = inverse_A[0][1] = A[0][3]*A[2][2]*A[3][1]+A[0][2]*A[2][1]*A[3][3]+A[0][1]*A[2][3]*A[3][2]-A[0][1]*A[2][2]*A[3][3]-A[0][2]*A[2][3]*A[3][1]-A[0][3]*A[2][1]*A[3][2];
    inverse_A[2][0] = inverse_A[0][2] = A[0][1]*A[1][2]*A[3][3]+A[0][2]*A[1][3]*A[3][1]+A[0][3]*A[1][1]*A[3][2]-A[0][3]*A[1][2]*A[3][1]-A[0][2]*A[1][1]*A[3][3]-A[0][1]*A[1][3]*A[3][2];
    inverse_A[0][3] = inverse_A[3][0] = A[0][3]*A[1][2]*A[2][1]+A[0][2]*A[1][1]*A[2][3]+A[0][1]*A[1][3]*A[2][2]-A[0][1]*A[1][2]*A[2][3]-A[0][2]*A[1][3]*A[2][1]-A[0][3]*A[1][1]*A[2][2];
    inverse_A[1][1] = A[0][0]*A[2][2]*A[3][3]+A[0][2]*A[2][3]*A[3][0]+A[0][3]*A[2][0]*A[3][2]-A[0][3]*A[2][2]*A[3][0]-A[0][2]*A[2][0]*A[3][3]-A[0][0]*A[2][3]*A[3][2];
    inverse_A[2][1] = inverse_A[1][2] = A[0][3]*A[1][2]*A[3][0]+A[0][2]*A[1][0]*A[3][3]+A[0][0]*A[1][3]*A[3][2]-A[0][0]*A[1][2]*A[3][3]-A[0][2]*A[1][3]*A[3][0]-A[0][3]*A[1][0]*A[3][2];
    inverse_A[3][1] = inverse_A[1][3] = A[0][0]*A[1][2]*A[2][3]+A[0][2]*A[1][3]*A[2][0]+A[0][3]*A[1][0]*A[2][2]-A[0][3]*A[1][2]*A[2][0]-A[0][2]*A[1][0]*A[2][3]-A[0][0]*A[1][3]*A[2][2];
    inverse_A[2][2] = A[0][0]*A[1][1]*A[3][3]+A[0][1]*A[1][3]*A[3][0]+A[0][3]*A[1][0]*A[3][1]-A[0][3]*A[1][1]*A[3][0]-A[0][1]*A[1][0]*A[3][3]-A[0][0]*A[1][3]*A[3][1];
    inverse_A[3][2] = inverse_A[2][3] = A[0][3]*A[1][1]*A[2][0]+A[0][1]*A[1][0]*A[2][3]+A[0][0]*A[1][3]*A[2][1]-A[0][0]*A[1][1]*A[2][3]-A[0][1]*A[1][3]*A[2][0]-A[0][3]*A[1][0]*A[2][1];
    inverse_A[3][3] = A[0][0]*A[1][1]*A[2][2]+A[0][1]*A[1][2]*A[2][0]+A[0][2]*A[1][0]*A[2][1]-A[0][2]*A[1][1]*A[2][0]-A[0][1]*A[1][0]*A[2][2]-A[0][0]*A[1][2]*A[2][1];

    double det_a = 0;
    det_a += A[0][0]*(A[1][1]*A[2][2]*A[3][3]+A[1][2]*A[2][3]*A[3][1]+A[1][3]*A[2][1]*A[3][2]-A[1][3]*A[2][2]*A[3][1]-A[1][2]*A[2][1]*A[3][3]-A[1][1]*A[2][3]*A[3][2]);
    det_a -= A[1][0]*(A[0][1]*A[2][2]*A[3][3]+A[0][2]*A[2][3]*A[3][1]+A[0][3]*A[2][1]*A[3][2]-A[0][3]*A[2][2]*A[3][1]-A[0][2]*A[2][1]*A[3][3]-A[0][1]*A[2][3]*A[3][2]);
    det_a += A[2][0]*(A[0][1]*A[1][2]*A[3][3]+A[0][2]*A[1][3]*A[3][1]+A[0][3]*A[1][1]*A[3][2]-A[0][3]*A[1][2]*A[3][1]-A[0][2]*A[1][1]*A[3][3]-A[0][1]*A[1][3]*A[3][2]);
    det_a -= A[3][0]*(A[0][1]*A[1][2]*A[2][3]+A[0][2]*A[1][3]*A[2][1]+A[0][3]*A[1][1]*A[2][2]-A[0][3]*A[1][2]*A[2][1]-A[0][2]*A[1][1]*A[2][3]-A[0][1]*A[1][3]*A[2][2]);

    for(int i=0; i<4; i++)
    {
        for(int j=0; j<4; j++)
        {
            inverse_A[i][j] /= det_a;
        }
    }

    coeff[0] = inverse_A[0][0]*B[0] + inverse_A[0][1]*B[1] + inverse_A[0][2]*B[2] + inverse_A[0][3]*B[3];
    coeff[1] = inverse_A[1][0]*B[0] + inverse_A[1][1]*B[1] + inverse_A[1][2]*B[2] + inverse_A[1][3]*B[3];
    coeff[2] = inverse_A[2][0]*B[0] + inverse_A[2][1]*B[1] + inverse_A[2][2]*B[2] + inverse_A[2][3]*B[3];
    coeff[3] = inverse_A[3][0]*B[0] + inverse_A[3][1]*B[1] + inverse_A[3][2]*B[2] + inverse_A[3][3]*B[3];

    //std::cout<<coeff[0]<<std::endl;
    //std::cout<<coeff[1]<<std::endl;
    //std::cout<<coeff[2]<<std::endl;
    //std::cout<<coeff[3]<<std::endl;

}

void Lane::half_lane_print(int size, int side)
{
    int x_base;
    cv::Mat print_img = cv::Mat::zeros(size, size/2, CV_8UC1);
    for(int i=0; i<size ;i++)
    {
        x_base = static_cast<int>(coeff[0]+coeff[1]*i+coeff[2]*i*i+coeff[3]*i*i*i);
        for(int j=x_base-2; j<= x_base+2; j++)
        {
            if(color == 0)
            {
                print_img.at<uchar>(i,j) = 128;
            }
            else
            {
                print_img.at<uchar>(i,j) = 255;
            }
        }
    }
    /*
    if(side == 0)
    {
        cv::namedWindow("left_fitting_img");
        imshow("left_fitting_img",print_img);
    }
    else if (side == 1)
    {
        cv::namedWindow("right_fitting_img");
        imshow("right_fitting_img",print_img);
    }*/
    
    //cv::waitKey();
}