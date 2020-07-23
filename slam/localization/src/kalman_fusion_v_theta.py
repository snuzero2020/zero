#!/usr/bin/python
import rospy
import time
import numpy as np
import matplotlib.pyplot as plt
from localization.msg import Data

class KalmanFusionVisualizer :
    def __init__(self):
        self._sub = rospy.Subscriber("/data", Data, self.callback)
        self._index=0
        self._times =[]
        self._theta =[]
        self._v =[]

    def callback(self, msg):
        self._index = self._index + 1
        self._times.append(time.time())
        self._theta.append(msg.theta)
        self._v.append(msg.v)

        if self._index%1000 == 0:
            plt.subplot(231)
            plt.plot(self._times, theta)
            plt.subplot(232)
            plt.plot(self._times, v)
            plt.pause(0.1)
            plt.savefig('KalmanFusion_v_theta.png', dpi=300)


if __name__=='__main__':
    rospy.init_node("KalmanFusion_v_theta")
    kalmanfusion = KalmanFusionVisualizer()
    rospy.spin()










