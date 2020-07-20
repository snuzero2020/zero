#include "lane_class.h"
#include "lane_fit_sliding_window.h"
#include <iostream>
#include <vector>

cv::Mat get_fits_by_sliding_window(cv::Mat img, int n_window = 10)
{
    int img_size = img.cols;
    int half_size = img_size/2;

    cv::Mat left_img = img(cv::Rect(0,0,half_size,img_size));
    cv::Mat right_img = img(cv::Rect(half_size,0,half_size,img_size));

//    cv::namedWindow("left_lane");
//	cv::imshow("left_lane", left_img);
//   cv::waitKey();

    //left_img
    int *white_left_hist = new int[half_size];
    int *yellow_left_hist = new int[half_size];

    for(int i=0; i<half_size; i++)
    {
        white_left_hist[i] = 0;
        yellow_left_hist[i] = 0;
    }

    
    Lane left_lane;
    // 0 : white, 1 : yellow

	
    for(int i=0; i< half_size; i++)
    {
        for(int j=0; j< img_size; j++)
        {
            if(left_img.at<uchar>(j,i) == 255)
            {
                yellow_left_hist[i]++;
            }
            else if(left_img.at<uchar>(j,i) == 128)
            {
                white_left_hist[i]++;
            }
        }
    }

//    for(int i=0; i< half_size; i++)
//    {
//        std::cout<<yellow_left_hist[i]<<" "<< white_left_hist[i]<<std::endl;
//    }

//	cv::namedWindow("left_lane");
//	cv::imshow("left_lane", left_img);

    int left_xbase;

//  std::cout<<hist_sum(yellow_left_hist, half_size)<<" "<<hist_sum(white_left_hist, half_size)<<std::endl;

    if(hist_sum(yellow_left_hist, half_size)>hist_sum(white_left_hist, half_size))
    {
        left_lane.set_color(1);
        left_xbase = return_xbase(yellow_left_hist, half_size);
    //    std::cout<<"yellow"<<std::endl;
    }
    else
    {   
        left_lane.set_color(0);
        left_xbase = return_xbase(white_left_hist, half_size);
    //    std::cout<<"white"<<std::endl;
    }

    int window_x_min, window_x_max;
    int window_y_min, window_y_max;
    int window_x_size, window_y_size;
    int window_x_num = 3;

    window_x_size = 8;
    window_y_size = img_size/n_window;

    cv::Mat left_temp_img = left_img.clone();
    cv::Mat right_temp_img = right_img.clone();
    cv::Mat temp_img = cv::Mat::zeros(img_size, img_size, CV_8UC1);
    //cv::namedWindow("left_lane");
	//cv::imshow("left_lane", left_temp_img);
    //cv::waitKey();

    //std::cout<<left_xbase<<std::endl;
    //std::cout<< window_x_size << " "<<window_y_size<<std::endl;

    vector<int> temp_x;
    vector<int> temp_y;

    for(int iter = 0; iter < n_window; iter++)
    {
    //////////////////////////////////////////////////////
        window_x_min = left_xbase - ((3*window_x_size)/2);
        window_x_max = left_xbase + ((3*window_x_size)/2);
        window_y_min = img_size - (iter+1)*window_y_size;
        window_y_max = img_size - (iter)*window_y_size;
        
        if(window_x_min < 0)
        {
            window_x_min = 0;
        }
        if(window_x_max > img_size)
        {
            window_x_max = img_size-1;
        }
        
        for(int i = window_x_min; i < window_x_max; i++)
        {
            for(int j = window_y_min; j<window_y_max; j++)
            {
                if(left_lane.get_color() == 0)
                {
                    if(left_img.at<uchar>(j,i) == 128)
                    {
                        temp_x.push_back(i);
                    }
                }
                else
                {
                    if(left_img.at<uchar>(j,i) == 255)
                    {
                        temp_x.push_back(i);
                    }
                }
            }
        }
        if(temp_x.size() != 0)
        {
            left_xbase = return_vector_xbase(temp_x);
        }
        
        if(temp_x.size() > 10)
        {
            window_x_num = 3;
            window_x_min = left_xbase - ((3*window_x_size)/2);
            window_x_max = left_xbase + ((3*window_x_size)/2);
            window_y_min = img_size - (iter+1)*window_y_size;
            window_y_max = img_size - (iter)*window_y_size;
        }
        else
        {
            window_x_num = 5;
            window_x_min = left_xbase - ((5*window_x_size)/2);
            window_x_max = left_xbase + ((5*window_x_size)/2);
            window_y_min = img_size - (iter+1)*window_y_size;
            window_y_max = img_size - (iter)*window_y_size;
        }
        
        temp_x.clear();
    ///////////////////////////////////////////////////////

        if(window_x_min < 0)
        {
            window_x_min = 0;
        }
        if(window_x_max > img_size)
        {
            window_x_max = img_size-1;
        }

        for(int i = window_x_min; i < window_x_max; i++)
        {
            for(int j = window_y_min; j<window_y_max; j++)
            {
                if(left_lane.get_color() == 0)
                {
                    if(left_img.at<uchar>(j,i) == 128)
                    {
                        left_lane.input_xy(i,j);
                    }
                }
                else
                {
                    if(left_img.at<uchar>(j,i) == 255)
                    {
                        left_lane.input_xy(i,j);
                    }
                }
            }
        }

        //imshow("left_img",left_img);
        
        if(window_x_num == 3)
        {
            if(window_x_min >= 0 && window_y_min >= 0)
            {
                cv::rectangle(left_temp_img, cv::Rect(window_x_min, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            if(window_x_min + window_x_size >= 0 && window_y_min >= 0)
            {
                cv::rectangle(left_temp_img, cv::Rect(window_x_min + window_x_size, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            if(window_x_min + 2*window_x_size >= 0 && window_y_min >= 0)
            {
                cv::rectangle(left_temp_img, cv::Rect(window_x_min + 2*window_x_size, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            //imshow("left_temp_img",left_temp_img);
            //left_temp_img = left_img.clone();
            //cv::waitKey();
        }
        else if(window_x_num == 5)
        {
            if(window_x_min >= 0 && window_y_min >= 0)
            {
                cv::rectangle(left_temp_img, cv::Rect(window_x_min, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            if(window_x_min + window_x_size >= 0 && window_y_min >= 0)
            {
                cv::rectangle(left_temp_img, cv::Rect(window_x_min + window_x_size, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            if(window_x_min + 2*window_x_size >= 0 && window_y_min >= 0)
            {
                cv::rectangle(left_temp_img, cv::Rect(window_x_min + 2*window_x_size, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            if(window_x_min + 3*window_x_size >= 0 && window_y_min >= 0)
            {
                cv::rectangle(left_temp_img, cv::Rect(window_x_min + 3*window_x_size, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            if(window_x_min + 4*window_x_size >= 0 && window_y_min >= 0)
            {
                cv::rectangle(left_temp_img, cv::Rect(window_x_min + 4*window_x_size, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            //imshow("left_temp_img",left_temp_img);
            //left_temp_img = left_img.clone();
            //cv::waitKey();
        }

        window_x_num = 3;
    }

    left_lane.get_coeff();
    //left_lane.half_lane_print(img_size, 0); // 0: left , 1: right

    //right_img

    int *white_right_hist = new int[half_size];
    int *yellow_right_hist = new int[half_size];

    for(int i=0; i<half_size; i++)
    {
        white_right_hist[i] = 0;
        yellow_right_hist[i] = 0;
    }

    //std::cout<<"1"<<std::endl;

    Lane right_lane;
    // 0 : white, 1 : yellow

	
    for(int i=0; i< half_size; i++)
    {
        for(int j=0; j< img_size; j++)
        {
            if(right_img.at<uchar>(j,i) == 255)
            {
                yellow_right_hist[i]++;
            }
            else if(right_img.at<uchar>(j,i) == 128)
            {
                white_right_hist[i]++;
            }
        }
    }

    int right_xbase;

    if(hist_sum(yellow_right_hist, half_size)>hist_sum(white_right_hist, half_size))
    {
        right_lane.set_color(1);
        right_xbase = return_xbase(yellow_right_hist, half_size);
    }
    else
    {   
        right_lane.set_color(0);
        right_xbase = return_xbase(white_right_hist, half_size);
    }

    //std::cout<<"2"<<std::endl;
    
    window_x_num = 3;

    window_x_size = 8;
    window_y_size = img_size/n_window;

    temp_x.clear();
    temp_y.clear();

    for(int iter = 0; iter < n_window; iter++)
    {
    //////////////////////////////////////////////////////
        window_x_min = right_xbase - ((3*window_x_size)/2);
        window_x_max = right_xbase + ((3*window_x_size)/2);
        window_y_min = img_size - (iter+1)*window_y_size;
        window_y_max = img_size - (iter)*window_y_size;

        if(window_x_min < 0)
        {
            window_x_min = 0;
        }
        if(window_x_max > img_size)
        {
            window_x_max = img_size-1;
        }

        for(int i = window_x_min; i < window_x_max; i++)
        {
            for(int j = window_y_min; j<window_y_max; j++)
            {
                if(right_lane.get_color() == 0)
                {
                    if(right_img.at<uchar>(j,i) == 128)
                    {
                        temp_x.push_back(i);
                    }
                }
                else
                {
                    if(right_img.at<uchar>(j,i) == 255)
                    {
                        temp_x.push_back(i);
                    }
                }
            }
        }

        
        if(temp_x.size() != 0)
        {
            right_xbase = return_vector_xbase(temp_x);
        }
        
        temp_x.clear();
    ///////////////////////////////////////////////////////

        for(int i = window_x_min; i < window_x_max; i++)
        {
            for(int j = window_y_min; j<window_y_max; j++)
            {
                if(right_lane.get_color() == 0)
                {
                    if(right_img.at<uchar>(j,i) == 128)
                    {
                        right_lane.input_xy(i,j);
                    }
                }
                else
                {
                    if(right_img.at<uchar>(j,i) == 255)
                    {
                        right_lane.input_xy(i,j);
                    }
                }
            }
        }

        
         if(window_x_num == 3)
        {
            if(window_x_min >= 0 && window_y_min >= 0)
            {
                cv::rectangle(right_temp_img, cv::Rect(window_x_min, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            if(window_x_min + window_x_size >= 0 && window_y_min >= 0)
            {
                cv::rectangle(right_temp_img, cv::Rect(window_x_min + window_x_size, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            if(window_x_min + 2*window_x_size >= 0 && window_y_min >= 0)
            {
                cv::rectangle(right_temp_img, cv::Rect(window_x_min + 2*window_x_size, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            //imshow("left_temp_img",left_temp_img);
            //left_temp_img = left_img.clone();
            //cv::waitKey();
        }
        else if(window_x_num == 5)
        {
            if(window_x_min >= 0 && window_y_min >= 0)
            {
                cv::rectangle(right_temp_img, cv::Rect(window_x_min, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            if(window_x_min + window_x_size >= 0 && window_y_min >= 0)
            {
                cv::rectangle(right_temp_img, cv::Rect(window_x_min + window_x_size, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            if(window_x_min + 2*window_x_size >= 0 && window_y_min >= 0)
            {
                cv::rectangle(right_temp_img, cv::Rect(window_x_min + 2*window_x_size, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            if(window_x_min + 3*window_x_size >= 0 && window_y_min >= 0)
            {
                cv::rectangle(right_temp_img, cv::Rect(window_x_min + 3*window_x_size, window_y_min, window_x_size, window_y_size), 50, 2);
            }
            if(window_x_min + 4*window_x_size >= 0 && window_y_min >= 0)
            {
                cv::rectangle(right_temp_img, cv::Rect(window_x_min + 4*window_x_size, window_y_min, window_x_size, window_y_size), 50, 2);
            }
        }

        window_x_num = 3;
    }

    ////////////////////
    //std::cout<<"1"<<std::endl;
    //cv::Mat before_mask_img = cv::Mat::zeros(img_size, img_size, CV_8UC1);
    for(int i=0; i < half_size; i++)
    {
        for(int j=0; j < img_size; j++)
        {
            //std::cout<<i<<" "<<j<<std::endl;
            temp_img.at<uchar>(j,i) = left_temp_img.at<uchar>(j,i);
            temp_img.at<uchar>(j,i+half_size) = right_temp_img.at<uchar>(j,i);
            //before_mask_img.at<uchar>(j,i) = left_temp_img.at<uchar>(j,i);
            //before_mask_img.at<uchar>(j,i+half_size) = right_temp_img.at<uchar>(j,i);
        }
    }
    //std::cout<<"2"<<std::endl;

    cv::namedWindow("before_mask_img");
    cv::imshow("before_mask_img",img);
    cv::namedWindow("window_img");
	cv::imshow("window_img",temp_img);

    //std::cout<<"3"<<std::endl;

    right_lane.get_coeff();
    //right_lane.half_lane_print(img_size, 1); // 0: left , 1: right
    
    cv::Mat return_img = cv::Mat::zeros(img_size, img_size, CV_8UC1);
    int pivot_left, pivot_right;
    for(int i=0; i<img_size ;i++)
    {
        pivot_left = static_cast<int>(left_lane.coeff[0]+left_lane.coeff[1]*i+left_lane.coeff[2]*i*i+left_lane.coeff[3]*i*i*i);
        pivot_right = static_cast<int>(right_lane.coeff[0]+right_lane.coeff[1]*i+right_lane.coeff[2]*i*i+right_lane.coeff[3]*i*i*i);
        
        // std::cout<<pivot_left<<std::endl;
        // std::cout<<pivot_right<<std::endl;

        if(pivot_left < pivot_right)
        {
        for(int j=0; j< pivot_left-2; j++)
        {
            if(0<=i && i<img_size && 0<=j && j<img_size)
            {
                return_img.at<uchar>(i,j) = 255;
            }
        }
        for(int j=pivot_left-2; j<= pivot_left+2; j++)
        {
            if(left_lane.color == 0)
            {
                if(0<=i && i<img_size && 0<=j && j<img_size)
                {
                    return_img.at<uchar>(i,j) = 128;
                }
            }
            else
            {
                if(0<=i && i<img_size && 0<=j && j<img_size)
                {
                    return_img.at<uchar>(i,j) = 255;
                }
            }
        }
        for(int j=pivot_right-2; j<= pivot_right+2; j++)
        {
            if(right_lane.color == 0)
            {
                if(0<=i && i<img_size && 0<=(j+half_size) && (j+half_size)<img_size)
                {
                    return_img.at<uchar>(i,j+half_size) = 128;
                }
            }
            else
            {
                if(0<=i && i<img_size && 0<=(j+half_size) && (j+half_size)<img_size)
                {
                    return_img.at<uchar>(i,j+half_size) = 255;
                }
            }
        }
        for(int j=pivot_right+3; j<=half_size; j++)
        {
            if(0<=i && i<img_size && 0<=(j+half_size) && (j+half_size)<img_size)
            {
                return_img.at<uchar>(i,j+half_size) = 255;
            }
        }

        int pivot_1, pivot_2, pivot_3, pivot_4;
        pivot_1 = ((pivot_left)*4 + (pivot_right + half_size)*1)/5;
        pivot_2 = ((pivot_left)*3 + (pivot_right + half_size)*2)/5;
        pivot_3 = ((pivot_left)*2 + (pivot_right + half_size)*3)/5;
        pivot_4 = ((pivot_left)*1 + (pivot_right + half_size)*4)/5;

        for(int j = pivot_left+3; j< pivot_1; j++)
        {
            if(0<=i && i<img_size && 0<=j && j<img_size)
            {
                return_img.at<uchar>(i,j) = 100;
            }
        }
        for(int j = pivot_1; j< pivot_2; j++)
        {
            if(0<=i && i<img_size && 0<=j && j<img_size)
            {
                return_img.at<uchar>(i,j) = 50;
            }
        }
        for(int j = pivot_2; j< pivot_3; j++)
        {
            if(0<=i && i<img_size && 0<=j && j<img_size)
            {
                return_img.at<uchar>(i,j) = 0;
            }
        }
        for(int j = pivot_3; j< pivot_4; j++)
        {
            if(0<=i && i<img_size && 0<=j && j<img_size)
            {
                return_img.at<uchar>(i,j) = 50;
            }
        }
        for(int j = pivot_4; j< pivot_right-2+half_size; j++)
        {
            if(0<=i && i<img_size && 0<=j && j<img_size)
            {
                return_img.at<uchar>(i,j) = 100;
            }
        }
        }
    }

    delete []white_left_hist;
    delete []yellow_left_hist;
    delete []white_right_hist;
    delete []yellow_right_hist;

    return return_img;
}

int hist_sum(int *array, int size)
{
    int sum = 0;
    for(int i=0; i<size; i++)
    {
        sum+=array[i];
    }
    return sum;
}

int return_xbase(int *array, int size)
{
    int i, xbase = 0;

    for(i=0; i<size; i++)
    {
        if(array[xbase]<array[i])
        {
            xbase = i;
        }
    }
    return xbase;
}

int return_vector_xbase(vector<int> x)
{
    int size = x.size();
    int sum = 0;
    for(int i=0; i<size; i++)
    {
        sum += x[i];
    }
    return static_cast<int>(sum/size);
}

cv::Point2i get_goal_point(cv::Mat lane_mask, int size)
{
    int count = 0;
    double sum = 0;
    for(int i=0; i<size ;i++)
    {
        if(lane_mask.at<uchar>(0,i) == 0)
        {
            count++;
            sum += i;
        }
    }

    cv::Point2i ptr;
    ptr.x = static_cast<int>(sum/count);
    ptr.y = 0;
    return ptr;
}

