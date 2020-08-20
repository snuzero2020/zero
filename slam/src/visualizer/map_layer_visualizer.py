#!/usr/bin/python
import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from std_msgs.msg import Int32


class MapLayerVisualizer:
    def __init__(self):
        self._sub = rospy.Subscriber("/recommended_velocity", Float64, self.callback_velocity)
        self._index = 0
        self._times = []
        self._recommended_velocity =[]

    def callback_velocity(self, msg):
        self._index = self._index + 1
        self._times.append(time.time())
        self._recommended_velocity.append(msg.data)
            
        if self._index % 300 == 0:
            start_range = self._index-600
            plt.plot(self._times[start_range: self._index], self._recommended_velocity[start_range: self._index])
            plt.ylim(0.00, 3.50)
            plt.pause(0.1)
            plt.clf()
            plt.savefig('sector_velocity_visualization.png', dpi=300)

if __name__=='__main__':
    rospy.init_node("map_layer_visualization")
    map_layer_visualizer = MapLayerVisualizer()
    rospy.spin()
