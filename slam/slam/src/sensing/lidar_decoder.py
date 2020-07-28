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
from geometry_msgs.msg import Point
from slam.msg import Lidar
import time



class LidarDecoder:
    def __init__(self):
        self._pub = rospy.Publisher("/points", Lidar, queue_size=1)
        self._sub = rospy.Subscriber("/velodyne_points", sensor_msgs.msg.PointCloud2, self.callback_lidar)
        self._distance_tolerance = 10.0

    def callback_lidar(self, msg):
        rt = Lidar()
        cloud_points = []
        cloud_channels = []
        for point in pc2.read_points(msg, field_names={"x","y","z","intensity","ring"}, skip_nans=True):
            if point[0] > 0 and point[3] >10 and math.sqrt(point[0]*point[0]+point[1]*point[1]+point[2]*point[2]) < self._distance_tolerance:
                p = Point()
                p.x = point[0]
                p.y = point[1]
                p.z = point[2]
                cloud_points.append(p)
                cloud_channels.append(point[4])
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        rt.header = h
        rt.count = len(cloud_points)
        rt.points = cloud_points
        rt.channels = cloud_channels
        self._pub.publish(rt)

if __name__ == '__main__':
    rospy.init_node("lidar_decoder")
    lidar_decoder = LidarDecoder()
    rospy.spin()
