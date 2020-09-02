#!/usr/bin/env python
import rospy
import struct
import numpy as np
import sys
import random
import math
import std_msgs.msg
import geometry_msgs.msg
import sensor_msgs
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from slam.msg import Lidar
from slam.msg import LidarPoint
from slam.msg import Clusters
from slam.msg import Cluster
from std_msgs.msg import Header
import time



class Pc2Converter:
    def __init__(self):
        self._pub = rospy.Publisher("/cluster_to_pc2", PointCloud2, queue_size=1)
        self._sub = rospy.Subscriber("/point_cloud_clusters", Clusters, self.callback)
        self._first_cluster_threshold = 100
        self._second_cluster_threshold = 200

    def callback(self, msg):
        points = []
        for cluster in msg.clusters:
            index = 0
            if cluster.count > self._first_cluster_threshold:
                index = index + 1
            if cluster.count > self._second_cluster_threshold:
                index = index + 1
            r = 0
            g = 0
            b = 0
            a = 255
            if index == 0:
                r = 255
            if index == 1:
                g = 255
            if index == 2:
                b = 255
            rgb = struct.unpack('I', struct.pack('BBBB',b,g,r,a))[0]
            for point in cluster.points:
                pt = [point.point_3d.x, point.point_3d.y, point.point_3d.z, rgb]
                points.append(pt)
        fields = [PointField('x',0,PointField.FLOAT32,1),
                  PointField('y',4,PointField.FLOAT32,1),
                  PointField('z',8,PointField.FLOAT32,1),
                  PointField('rgba',12,PointField.UINT32,1)]
        header = Header()
        header.frame_id = "map"
        rt = pc2.create_cloud(header, fields, points)
        self._pub.publish(rt)

if __name__ == '__main__':
    rospy.init_node("cluster_to_pc2")
    pc2_converter = Pc2Converter()
    rospy.spin()
