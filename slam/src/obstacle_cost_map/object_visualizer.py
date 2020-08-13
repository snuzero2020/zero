#!/usr/bin/env python
import rospy
import rospkg
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
from slam.msg import Clusters
import time
import matplotlib.pyplot as plt

class ObjectVisualizer:
    def __init__(self):
        self._sub = rospy.Subscriber("/point_cloud_clusters", Clusters, self.callback)
        self._count = 0
        rospack = rospkg.RosPack()
        self._save_path = rospack.get_path('slam')+'/result/'


    def callback(self,msg):
        self._count = self._count+1
        clusters = msg.clusters
        index = 0
        X = []
        Y = []
        C = []
        for cluster in clusters:
            index = index + 1
            for point in cluster.points_2d:
                X.append(point.x)
                Y.append(point.y)
                C.append(index)
        plt.scatter(X,Y,c=C,s=2)
        plt.xlim([0,7])
        plt.ylim([-7,7])
        file_name = self._save_path+'object_detector_{}'.format(self._count)
        plt.savefig(file_name,dpi=300)
        plt.close()


if __name__ == '__main__':
    rospy.init_node("object_visualizer")
    object_visualizer = ObjectVisualizer()
    rospy.spin()
