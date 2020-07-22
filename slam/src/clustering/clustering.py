#!/usr/bin/env python
import rospy
import numpy as np
import sys
import random
import math
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs
import sensor_msgs.point_cloud2 as pc2
from slam.msg import Lidar
from slam.msg import Cluster
import time
import matplotlib.pyplot as plt



class Clustering:
    def __init__(self):
        self._pub_2d_obstacle_points = rospy.Publisher("/2d_obstacle_points", Cluster, queue_size=1)
        self._pub_3d_obstacle_points = rospy.Publisher("/3d_obstacle_points", Cluster, queue_size=1)
        self._sub = rospy.Subscriber("/points", Lidar, self.callback_points)
        self._iteration = 30
        self._remove_tolerance = 5.0
        self._plane_tolerance = 0.05
        self._clustering_tolerance = 0.1
        self._lidar_angle = 18 # unit : degree
        self._lidar_height = 1.25 # unit : m
        self._count = 0
    
    def projection(self, point, plane_config):
        a = plane_config['a']
        b = plane_config['b']
        c = plane_config['c']
        d = plane_config['d']
        t = (point[0]*a + point[1]*b + point[2]*c + d)/(a*a+b*b+c*c)
        return np.array([point[0]-a*t, point[1]-b*t, point[2]-c*t]).reshape(-1,1)
    
    
    def point_norm(self, point):
        return math.sqrt(point[0]*point[0] + point[1]*point[1] + point[2]*point[2])
    
    
    def select_candidate(self, points):
        candidate_points = []
        for i, point in enumerate(points):
            if -point.x * math.sin(self._lidar_angle*math.pi/180) + point.z*math.cos(self._lidar_angle*math.pi/180) < -self._lidar_height + self._plane_tolerance * 4:
                candidate_points.append(i)
        return candidate_points
    
    def ransac_plane(self, points, candidate_points, iteration, distance_tolerance):
        """
        In this function, we picks random 3 points from points list.
        And calculate the plane equation with these 3 points then count the number of points which are near from this plane
        Repeating certain iteration, return inliers and plane configuartion with maximum the number of inliers points
        """
        inliersResult = {}
        inliersResult = set()
        plane_config = {'a':0.0, 'b':0.0, 'c':0.0, 'd':0.0}
        while iteration:
            inliers = {}
            inliers = set()
            # Pick 3 points randomly
            while len(inliers) <3:
                inliers.add(candidate_points[random.randint(0,len(candidate_points)-1)])
            inliers_iter = inliers.__iter__()
            itr = next(inliers_iter)
            x1 = points[itr].x
            y1 = points[itr].y
            z1 = points[itr].z
            itr = next(inliers_iter)
            x2 = points[itr].x
            y2 = points[itr].y
            z2 = points[itr].z
            itr = next(inliers_iter)
            x3 = points[itr].x
            y3 = points[itr].y
            z3 = points[itr].z
            # Get Normal Vector by Cross Product
            a = ((y2-y1)*(z3-z1) - (y3-y1)*(z2-z1))
            b = ((z2-z1)*(x3-x1) - (x2-x1)*(z3-z1))
            c = ((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1))
            d = -(a*x1 + b*y1 + c*z1)
            if c<0:
                continue
            for i in range(len(points)):
                # Not consider three points already picked
                if i in inliers:
                    continue
                x4 = points[i].x
                y4 = points[i].y
                z4 = points[i].z
                # Distance between picked point and the plane
                dist = math.fabs(a*x4 + b*y4 + c*z4 +d) / math.sqrt(a*a + b*b + c*c)
                if dist < distance_tolerance:
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
        lidar_y = np.cross([plane_config['a'],plane_config['b'],plane_config['c']],[lidar_x[0][0],lidar_x[1][0],lidar_x[2][0]]) 
        #lidar_y = self.projection([0,1,0],plane_config)
        lidar_y = lidar_y.reshape(-1,1)
        #print(np.shape(lidar_y))
        lidar_x = lidar_x / self.point_norm(lidar_x)
        lidar_y = lidar_y / self.point_norm(lidar_y)
        L = np.hstack((lidar_x, lidar_y)) # lidar matrix, we will calculate (LtL)^-1Lt
        L = np.matmul(np.linalg.inv(np.matmul(np.transpose(L),L)),np.transpose(L))
        for point in points:
            projected_point = self.projection([point.x,point.y,point.z], plane_config)
            projected_point = np.matmul(L,projected_point-lidar_position)
            projected_points.append([projected_point[0][0], projected_point[1][0]])
        return projected_points
        #return np.matmul(L,np.transpose(np.array(points))).tolist()
    
    """
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
                if len(cluster)>5:
                    clusters.append(cluster)
        return clusters
    """
    
    def callback_points(self, msg):
        self._count = self._count +1
        # load pointclouds data
        print("1")
        times = []
        times.append(time.time())
        cloud_points = msg.points
        cloud_channels = msg.channels
        print("2")
        candidate_points = self.select_candidate(msg.points)
        print("# of candidate points : {}",format(len(candidate_points)))

        inliers, plane_config = self.ransac_plane(cloud_points, candidate_points, self._iteration, self._plane_tolerance)
        print("3")
        
        filtered_points, filtered_channels = self.filtering_points(cloud_points, cloud_channels, inliers)
        print("# of filtered_points :{}",format(len(filtered_points)))
        projected_points = self.projecting_points(filtered_points, plane_config)
        
        #tree = KDTree(np.array(projected_points))
        #clusters = self.euclidean_clustering(np.array(projected_points), tree, self._clustering_tolerance)
        #print("# of clusters : {}",format(len(clusters)))

        X=[]
        Y=[]
        
        publish_clusters = []
        publish_channels = []
        publish_points = []
        publish_projected_points = []
        
        publish_2d_msg = Cluster()
        publish_3d_msg = Cluster()
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        
        publish_2d_msg.header = h
        publish_3d_msg.header = h
        
        publish_2d_msg.is_3d = False
        publish_3d_msg.is_3d = True
        
        for point in projected_points:
            p = geometry_msgs.msg.Point()
            p.x = point[0]
            p.y = point[1]
            publish_projected_points.append(p)
        
        """
        cluster_idx = 0
        count = 0
        for cluster in clusters:
            cluster_idx =  cluster_idx + 1
            for idx in cluster:
                publish_clusters.append(cluster_idx)
                publish_channels.append(filtered_channels[idx])
                point = geometry_msgs.msg.Point()
                point.x = filtered_points[idx].x
                point.y = filtered_points[idx].y
                point.z = filtered_points[idx].z
                publish_points.append(point)
                point = geometry_msgs.msg.Point()
                point.x = projected_points[idx][0]
                point.y = projected_points[idx][1]
                point.z = 0
                publish_projected_points.append(point)
                count = count + 1
                X.append(projected_points[idx][0])
                Y.append(projected_points[idx][1])
        """
        count = len(filtered_points)
        publish_2d_msg.count = count
        publish_3d_msg.count = count
        publish_2d_msg.points = publish_projected_points
        publish_3d_msg.points = filtered_points
        publish_2d_msg.channels = filtered_channels
        publish_3d_msg.channels = filtered_channels
        self._pub_2d_obstacle_points.publish(publish_2d_msg)
        self._pub_3d_obstacle_points.publish(publish_3d_msg)
        
        times.append(time.time())
        for i in range(len(times)):
            if i ==0:
                continue
            print(times[i]-times[i-1])
        print(times[len(times)-1]-times[0])
        for i in range(len(projected_points)):
            X.append(projected_points[i][0])
            Y.append(projected_points[i][1])
        plt.scatter(X,Y, s=2)
        plt.xlim([0,7])
        plt.ylim([-7,7])
        file_name = '/home/jeongwoooh/clustering/clustering_{}'.format(self._count)
        plt.savefig(file_name,dpi=300)
        #plt.pause(0.1)
        plt.close()


if __name__ == '__main__':
    rospy.init_node("clustering")
    clustering = Clustering()
    rospy.spin()
