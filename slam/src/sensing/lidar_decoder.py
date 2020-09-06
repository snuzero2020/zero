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
from slam.msg import LidarPoint
import time



class LidarDecoder:
    def __init__(self):
        self._pub = rospy.Publisher("/points", Lidar, queue_size=1)
        self._sub = rospy.Subscriber("/velodyne_points", sensor_msgs.msg.PointCloud2, self.callback_lidar)
        self._distance_tolerance = 12.0
        # self._lidar_angle = 0
        # self._lidar_height = 1.334
        self._lidar_angle = 18.48311
        self._lidar_height = 1.23690

    def callback_lidar(self, msg):
        rt = Lidar()
        cloud_points = []
        for point in pc2.read_points(msg, field_names={"x","y","z","intensity","ring"}, skip_nans=True):
            p = LidarPoint()
            p.point_2d.x = point[0]*math.cos(self._lidar_angle*math.pi/180) + point[2]*math.sin(self._lidar_angle*math.pi/180)
            p.point_2d.y = point[1]
            p.point_2d.z = 0
            p.point_3d.x = point[0]
            p.point_3d.y = point[1]
            p.point_3d.z = point[2]
            p.channel = point[4]
            if p.point_2d.x > 0 and point[3] > 10 and math.sqrt(p.point_2d.x*p.point_2d.x+p.point_2d.y*p.point_2d.y) < self._distance_tolerance:
                cloud_points.append(p)
            """
            if point[0] > 0 and point[3] >10 and math.sqrt(point[0]*point[0]+point[1]*point[1]+point[2]*point[2]) < self._distance_tolerance:
                p = LidarPoint()
                p.point_2d.x = point[0]*math.cos(self._lidar_angle*math.pi/180) + point[2]*math.sin(self._lidar_angle*math.pi/180) 
                p.point_2d.y = point[1]
                p.point_2d.z = 0
                p.point_3d.x = point[0]
                p.point_3d.y = point[1]
                p.point_3d.z = point[2]
                p.channel = point[4]
                cloud_points.append(p)
            """
        h = std_msgs.msg.Header()
        h.stamp = rospy.Time.now()
        rt.header = h
        rt.count = len(cloud_points)
        print(rt.count)
        rt.points = cloud_points
        self._pub.publish(rt)

if __name__ == '__main__':
    rospy.init_node("lidar_decoder")
    lidar_decoder = LidarDecoder()
    rospy.spin()
