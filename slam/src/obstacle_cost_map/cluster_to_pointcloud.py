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
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import Point
from slam.msg import Lidar
from slam.msg import LidarPoint
from slam.msg import Clusters
from slam.msg import Cluster
import time



class Pc2Converter:
    def __init__(self):
        self._pub = rospy.Publisher("/cluster_to_pc2", PointCloud2, queue_size=1)
        self._sub = rospy.Subscriber("/point_cloud_clusters", slam::Clusters, self.callback)

    def callback(self, msg):
        points = []
        for cluster in msg.clusters:
            for point in cluster.points:
                pt = [point.point_3d.x, point.point_3d.y, point.point_3d.z]
                points.append(pt)
        fields = [PointField('x',0,PointField.FLOAT32,1),
                  PointField('y',4,PointField.FLOAT32,1),
                  PointField('z',8,PointField.FLOAT32,1)]
        header = Header()
        header.frame_id = "map"
        rt = point_cloud2.create_cloud(header, fields, points)
        self._pub.publish(rt)

if __name__ == '__main__':
    rospy.init_node("cluster_to_pc2")
    pc2_converter = Pc2Converter()
    rospy.spin()
