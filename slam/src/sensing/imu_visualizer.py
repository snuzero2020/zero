#!/usr/bin/python
import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import Imu


class Imu_Visualizer:
    def __init__(self):
        self._sub = rospy.Subscriber("/imu/data", Imu, self.callback)
        self._index=0
        self._times =[]
        self._local_ax =[]
        self._local_ay =[]
        self._local_az =[]
        self._angular_vx=[]
        self._angular_vy=[]
        self._angular_vz=[]

    def callback(self, msg):
        self._index = self._index + 1
        self._times.append(time.time())
        self._local_ax.append(msg.linear_acceleration.x)
        self._local_ay.append(msg.linear_acceleration.y)
        self._local_az.append(msg.linear_acceleration.z)
        self._angular_vx.append(msg.angular_velocity.x)
        self._angular_vy.append(msg.angular_velocity.y)
        self._angular_vz.append(msg.angular_velocity.z)
       
        if self._index%1000 == 0:
            plt.subplot(231)
            plt.plot(self._times, self._local_ax)
            plt.subplot(232)
            plt.plot(self._times, self._local_ay)
            plt.subplot(233)
            plt.plot(self._times, self._local_az)
            plt.subplot(234)
            plt.plot(self._times, self._angular_vx)
            plt.subplot(235)
            plt.plot(self._times, self._angular_vy)
            plt.subplot(236)
            plt.plot(self._times, self._angular_vz)
            plt.pause(0.1)
            plt.savefig('imu_visualization.png', dpi=300)
            

if __name__=='__main__':
    rospy.init_node("imu_visualization")
    imu_visualizer = Imu_Visualizer()
    rospy.spin()
