import rospy
import numpy as np
import sys
import random
import math
from sklearn.neighbors import KDTree
import sensor_msgs.point_cloud2 as pc2
from clustering.msg import Points



class Clustering:
    def __init__(self):
        self._pub_2d_obstacle_points = rospy.Publisher("/2d_obstacle_points", Points, queue_size=1)
        self._pub_3d_obstacle_points = rospy.Publisher("/3d_obstacle_points", Points, queue_size=1)
        self._sub = rospy.Subscriber("/velodyne_points", self.callback_point_clouds)
        self._iteration = 100
        self._remove_tolerance = 10.0
        self._plane_tolerance = 0.05
        self._clustering_tolerance = 0.01
        

    def load_point_clouds(self, msg, distance_tolerance):
        cloud_points = []
        cloud_channels = []
        for point in pc2.read_points(msg, field_names={"x","y","z","intensity","ring"}, skip_nans=True):
            if point[0] > 0 and math.sqrt(point[0]*point[0]+point[1]*point[1]+point[2]*point[2]) < distance_tolerance:
                cloud_points.append([point[0],point[1],point[2]])
                cloud_channels.append(point[4])
        return cloud_points, cloud_channels


    def remove_plane(self, points, iteration, distance_tolerance):

    
    def filtering_points(self, points, inliers):

    
    def projecting_points(self, points, plane_config):


    def euclidean_clustering(self, points, tree, distance_tolerance):


    def callback_point_clouds(self, msg):
        cloud_points, cloud_channels = self.load_point_clouds(msg, self._remove_tolerance)
        inliers, plane_config = self.remove_plane(cloud_points, self._iteration, self._plane_tolerance)
        filtered_points, filtered_channels = self.filtering_points(cloud_points, inliers)
        projected_points = self.




if __name__ == '__main__':
    rospy.init_node("clustering")
    clustering = Clustering()
    rospy.spin()