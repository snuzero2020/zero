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

class ObjectVisualizer:
    def __init__(self):
        self._sub = rospy.Subscriber("/2d_obstacle_clouds", Cluster, self.callback)
        self._count = 0


    def callback(self,msg):
        self._count = self._count+1
        points = msg.points
        clusters = msg.clusters
        X = []
        Y = []
        C = []
        for point in points:
            X.append(point.x)
            Y.append(point.y)
        for c in clusters:
            C.append(c)
        plt.scatter(X,Y,c=C,s=2)
        plt.xlim([0,7])
        plt.ylim([-7,7])
        file_name = '/home/jeongwoooh/clustering/object_detector_{}'.format(self._count)
        plt.savefig(file_name,dpi=300)
        plt.close()


if __name__ == '__main__':
    rospy.init_node("object_visualizer")
    object_visualizer = ObjectVisualizer()
    rospy.spin()
