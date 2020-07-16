#!/usr/bin/env python
import rospy
import time
import std_msgs.msg
import matplotlib.pyplot as plt


class KalmanVisualization:
    def __init__(self):
        self._sub = rospy.Subscriber("/estimated_err",std_msgs.msg.Float64MultiArray, self.callback)
        self._times = []
        self._err_x = []
        self._err_y = []
        self._err_u = []
        self._err_v = []
        self._err_w = []
        self._init_time = time.time()

    def callback(self, msg):
        self._times.append(time.time()-self._init_time)
        self._err_x.append(msg.data[0])
        self._err_y.append(msg.data[1])
        self._err_u.append(msg.data[2])
        self._err_v.append(msg.data[3])
        self._err_w.append(msg.data[4])
        
        plt.subplot(231)
        plt.plot(self._times, self._err_x)
        plt.subplot(232)
        plt.plot(self._times, self._err_y)
        plt.subplot(233)
        plt.plot(self._times, self._err_u)
        plt.subplot(234)
        plt.plot(self._times, self._err_v)
        plt.subplot(235)
        plt.plot(self._times, self._err_w)
        
        plt.pause(0.01)
    



if __name__=='__main__':
    rospy.init_node("kalman_visualization")
    kalman_visualization = KalmanVisualization()
    rospy.spin()
