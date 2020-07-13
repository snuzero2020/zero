#include "lane_math.h"
#include <cmath>

int x_and_y_pow(int x, int x_n, int y, int y_n)
{
    if(x_n == 0 && y_n !=0)
    {
        return pow(y, y_n);
    }
    else if(x_n != 0 && y_n ==0)
    {
        return pow(x, x_n);
    }
    else
    {
        return pow(x, x_n)*pow(y, y_n);
    }
}