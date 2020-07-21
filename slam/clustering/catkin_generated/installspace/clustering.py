#!/usr/bin/env python2
import rospy
import numpy as np
import sys
import random
import math
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs
from sklearn.neighbors import KDTree
import sensor_msgs.point_cloud2 as pc2
from localization.msg import Points
import time



class Clustering:
    def __init__(self):
        self._pub_2d_obstacle_points = rospy.Publisher("/2d_obstacle_points", Points, queue_size=1)
        self._pub_3d_obstacle_points = rospy.Publisher("/3d_obstacle_points", Points, queue_size=1)
        self._sub = rospy.Subscriber("/velodyne_points", sensor_msgs.msg.PointCloud2, self.callback_point_clouds)
        self._iteration = 100
        self._remove_tolerance = 10.0
        self._plane_tolerance = 0.05
        self._clustering_tolerance = 0.01
        
    
    def projection(self, point, plane_config):
        a = plane_config['a']
        b = plane_config['b']
        c = plane_config['c']
        d = plane_config['d']
        t = (point[0]*a + point[1]*b + point[2]*c + d)/(a*a+b*b+c*c)
        return np.array([point[0]-a*t, point[1]-b*t, point[2]-c*t]).reshape(-1,1)
    
    
    def point_norm(self, point):
        return math.sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2])
    
    
    def load_point_clouds(self, msg, distance_tolerance):
        cloud_points = []
        cloud_channels = []
        for point in pc2.read_points(msg, field_names={"x","y","z","intensity","ring"}, skip_nans=True):
            if point[0] > 0 and math.sqrt(point[0]*point[0]+point[1]*point[1]+point[2]*point[2]) < distance_tolerance:
                cloud_points.append([point[0],point[1],point[2]])
                cloud_channels.append(point[4])
        return cloud_points, cloud_channels
    
    
    def ransac_plane(self, points, iteration, distance_tolerance):
        inliersResult = {}
        inliersResult = set()
        plane_config = {'a':0.0, 'b':0.0, 'c':0.0, 'd':0.0}
        while iteration:
            inliers = {}
            inliers = set()
            while len(inliers) <3:
                inliers.add(random.randint(0,len(points)-1))
            inliers_iter = inliers.__iter__()
            itr = next(inliers_iter)
            x1 = points[itr][0]
            y1 = points[itr][1]
            z1 = points[itr][2]
            itr = next(inliers_iter)
            x2 = points[itr][0]
            y2 = points[itr][1]
            z2 = points[itr][2]
            itr = next(inliers_iter)
            x3 = points[itr][0]
            y3 = points[itr][1]
            z3 = points[itr][2]
            # Get Normal Vector by Cross Product
            a = ((y2-y1)*(z3-z1) - (y3-y1)*(z2-z1))
            b = ((z2-z1)*(x3-x1) - (x2-x1)*(z3-z1))
            c = ((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1))
            d = -(a*x1 + b*y1 + c*z1)
            for i in range(len(points)):
                # Not consider three points already picked
                if i in inliers:
                    continue
            x4 = points[i][0]
            y4 = points[i][1]
            z4 = points[i][2]
            # Distance between picked point and the plane
            dist = math.fabs(a*x4 + b*y4 + c*z4 +d) / math.sqrt(a*a + b*b + c*c)
            if dist <= distance_tolerance:
                inliers.add(i)
            if len(inliers) > len(inliersResult):
                inliersResult = inliers
                plane_config['a'] = a
                plane_config['b'] = b
                plane_config['c'] = c
                plane_config['d'] = d
            iteration -= 1
        return inliersResult, plane_config
    
    
    def filtering_points(self, points, channels, inliers):
        point_check= [False for i in range(len(points))]
        filtered_points = []
        filtered_channels = []
        for i in inliers:
            point_check[i]=True
        for i in range(len(points)):
            if point_check[i]:
                continue
            filtered_points.append(points[i])
            filtered_channels.append(channels[i])
        return filtered_points, filtered_channels
        
    
    def projecting_points(self, points, plane_config):
        projected_points = []
        lidar_position = self.projection([0,0,0],plane_config)
        lidar_x = self.projection([1,0,0],plane_config)
        lidar_y = self.projection([0,1,0],plane_config)
        lidar_x = lidar_x / self.point_norm(lidar_x)
        lidar_y = lidar_y / self.point_norm(lidar_y)
        L = np.hstack((lidar_x, lidar_y)) # lidar matrix, we will calculate (LtL)^-1Lt
        L = np.matmul(np.linalg.inv(np.matmul(np.transpose(L),L)),np.transpose(L))
        for point in points:
            projected_point = self.projection(point, plane_config)
            projected_point = np.matmul(L,projected_point-lidar_position)
            projected_points.append([projected_point[0][0], projected_point[1][0]])
        return projected_points
        #return np.matmul(L,np.transpose(np.array(points))).tolist()
    
    
    def euclidean_clustering(self, points, tree, distance_tolerance):
        clusters = []
        processed = [False for i in range(len(points))]
        i = 0
        while i < len(points):
            storage = []
            cluster = []
            if not processed[i]:
                storage.append(i)
                while len(storage) >0:
                    index = storage.pop()
                    nearest_index, _ = tree.query_radius([points[index]], r=distance_tolerance, return_distance=True)
                    cluster.append(index)
                    i = i+1
                    processed[index]=True
                    for point in range(len(nearest_index[0])):
                        if not processed[point]:
                            storage.append(point)
                if len(cluster)>10:
                    clusters.append(cluster)
        return clusters
    
    
    def callback_point_clouds(self, msg):
        times = []
        times.append(time.time())
        cloud_points, cloud_channels = self.load_point_clouds(msg, self._remove_tolerance)
        times.append(time.time())
        inliers, plane_config = self.ransac_plane(cloud_points, self._iteration, self._plane_tolerance)
        times.append(time.time())
        filtered_points, filtered_channels = self.filtering_points(cloud_points, cloud_channels, inliers)
        times.append(time.time())
        projected_points = self.projecting_points(filtered_points, plane_config)
        times.append(time.time())
        tree = KDTree(np.array(projected_points))
        clusters = self.euclidean_clustering(np.array(projected_points), tree, self._clustering_tolerance)
        times.append(time.time())
        publish_clusters = []
        publish_channels = []
        publish_points = []
        publish_projected_points = []
        
        publish_2d_msg = Points()
        publish_3d_msg = Points()
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        
        publish_2d_msg.header = h
        publish_3d_msg.header = h
        
        publish_2d_msg.is_3d = False
        publish_3d_msg.is_3d = True
        
        cluster_idx = 0
        count = 0
        for cluster in clusters:
            cluster_idx =  cluster_idx + 1
            for idx in cluster:
                publish_clusters.append(cluster_idx)
                publish_channels.append(filtered_channels[idx])
                point = geometry_msgs.msg.Point()
                point.x = filtered_points[idx][0]
                point.y = filtered_points[idx][1]
                point.z = filtered_points[idx][2]
                publish_points.append(point)
                point = geometry_msgs.msg.Point()
                point.x = projected_points[idx][0]
                point.y = projected_points[idx][1]
                point.z = 0
                publish_projected_points.append(point)
                count = count + 1
        
        publish_2d_msg.count = count
        publish_3d_msg.count = count
        publish_2d_msg.points = publish_projected_points
        publish_3d_msg.points = publish_points
        publish_2d_msg.clusters = publish_clusters
        publish_3d_msg.clusters = publish_clusters
        publish_2d_msg.channels = publish_channels
        publish_3d_msg.channels = publish_channels
        self._pub_2d_obstacle_points.publish(publish_2d_msg)
        self._pub_3d_obstacle_points.publish(publish_3d_msg)
        times.append(time.time())
        for i in range(len(times)):
            if i ==0:
                continue
            print(times[i]-times[i-1])
        print(times[len(times)-1]-times[0])
        print(len(projected_points))


if __name__ == '__main__':
    rospy.init_node("clustering")
    rospy.loginfo("1")
    clustering = Clustering()
    rospy.spin()
