#include <iostream>
#include <vector>
#include <map>
#include <random>
#include <math.h>
#include <MatrixAlgebra.h>
#include <KDTree.hpp>

bool Contains(const std::vector<int> &list, int x)
{
    return std::find(list.begin(), list.end(), x) != list.end();
}

int ransac_plane(std::vector<std::vector<std::double>> points, std::int iteration, std::float distance_tolerance, std::vector<std::int>& inliersResult, map<std::char, std::double>& plane_config){
    inliersResult.clear();
    
    double x1, x2, x3, y1, y2, y3, z1, z2, z3, x4, y4, z4, a, b, c, d;
    
    plane_config['a'] = 0.0;
    plane_config['b'] = 0.0;
    plane_config['c'] = 0.0;
    plane_config['d'] = 0.0;
    
    std::vector<std::int> inliers;
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<int> dis(0, points.size());
    
    while(iteration>0){
        inliers.clear();
        //pick 3 points randomly
        while(inliers.size()<3){
            inliers.insert(dis(gen()));
        }
        itr = inliers[0];
        x1 = points[itr][0];
        y1 = points[itr][1];
        z1 = points[itr][2];
        itr = inliers[1];
        x2 = points[itr][0];
        y2 = points[itr][1];
        z2 = points[itr][2];
        itr = inliers[2];
        x3 = points[itr][0];
        y3 = points[itr][1];
        z3 = points[itr][2];
        a = ((y2-y1)*(z3-z1) - (y3-y1)*(z2-z1));
        b = ((z2-z1)*(x3-x1) - (x2-x1)*(z3-z1));
        c = ((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1));
        d = -(a*x1 + b *y1 + c*z1);
        
        for(int i = 0; i < points.size(); i++){
            if(Contains(inliers, i)) continue;
            x4 = points[i][0];
            y4 = points[i][1];
            z4 = points[i][2];
            
            double dist = gabs(a*x4 + b*y4 + c*z4 +d)/math.sqrt(a*a + b*b + c*c);
            
            if(dist <= distance_tolerance) inliers.insert(i);
            if(inliers.size()>inliersResult.size()){
                inliersResult = inliers;
                plane_config['a'] = a;
                plane_config['b'] = b;
                plane_config['c'] = c;
                plane_config['d'] = d;
            }
        }
        iteration -= 1;
    }
    result 0;
}

std::vector<std::double> projection(std::vector<std::vector<std::double>> points, map<std::char, std::double> plane_config){
    double a, b, c, d, t;
    a = plane_config['a'];
    b = plane_config['b'];
    c = plane_config['c'];
    d = plane_config['d'];
    t = (points[0]*a + points[1]*b + points[2]*c + d)/(a*a+b*b+c*c));
    std::vector<std::double> proj_point = [points[0]-a*t, point[1]-b*t, point[2]-c*t];
    return proj_point;
}

double point_norm(std::vector<std::vector<std::double>> points){
    return sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2]);
}

int filering_points(std::vector<std::vector<std::double>> points, std::vector<std::int> channels, std::vector<std::int> inliers, std::vector<std::vector<std::double>>& filtered_points, std::vector<std::int>& filtered_channels){
    
    std::vector<std::int> point_check;
    point_check{0};
    
    filtered_points.clear;
    filtered_channels.clear;
    
    for(int i=0; i < inliers.size(); i++) point_check[inliers[i]] = 1;
    for(int i=0; i < points.size(); i++){
        if(point_check[i] == 1) continue;
        filtered_points.insert(points[i]);
        filtered_channels.insert(channels[i]);
    }
    
    return 0;
}

std::vector<std::vector<std::double>> projecting_points(std::vector<std::vector<std::double>> points, map<std::char, std::double> plane_config){
    std::vector<std::vector<std::double>> projected_points;
    projected_points.clear();
    
    std::vector<std::double> lidar_position = projection([0,0,0], plane_config);
    std::vector<std::double> lidar_x = projection([1,0,0],plane_config);
    std::vector<std::double> lidar_y = projection([0,1,0],plane_config);
    
    lidar_x /= point_norm(lidar_x);
    lidar_y /= point_norm(lidar_y);
    
    
}

std:;vector<std::vector<std::int>> eucledean_clustering(std::vector<std::vector<double>> points, 'trees datatype', std::double distance_tolerance){
    std::vector<std::int> clusters;
    std::vector<std::bool> processed;
    std::vector<std::int> storage;

    for(int i=0; i < points.size(); i++) processed[i] = False;

    for(int i=0; i<points.size(); i++){

    }
}

int callback_point_clouds(std::vector<std::vector<std::double>> points, std::vector<std::int> channels){
    int iteration{100};
    double distance_tolerance{0.05};

    std::vector<std::int> inliersResult;
    map<std::char, std::double> plane_config;

    ransac_plane(points, iteration, distance_tolerance);

    std::vector<std::vector<std::double>> filtered_points;
    std::vector<std::int> filtered_channels;

    filtering_points(points, channels, inliersResult);

    std::vector<std::vector<std::double>> projected_points;

    projecting_points(filtered_points, plane_config);

    //have to insert making a KDTree
    eucledean_clustering()

}

std::vector<std::vector<std::int>> eucledean_clustering(std::vector<std::vector<std::double>> points,

// projection O point_norm O ransac_plane O filtering_points O projecting_points X euclidean_clustering X callback_point_clouds X
