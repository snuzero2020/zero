#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from core_msgs.msg import VehicleState
from slam.msg import Data
from matplotlib import pyplot as plt
import sys, select, termios, tty
from std_msgs.msg import Float64
from core_msgs.msg import Encoderfilter

def init():
    while not rospy.is_shutdown():
        print("control_test")
        rospy.init_node('control_test', anonymous = True)
        vehiclestate = getVehicleState()
        #rospy.Subscriber("filtered_data", Data, vehiclestate.callback)
        rospy.Subscriber("filter_encoder_data",Encoderfilter, vehiclestate.callback)
        rospy.spin()

count = 0
start_time = 0.0
class getVehicleState():
    def __init__(self):
        self.speed = []
        self.steer = []
        self.time = []
        self.number = 0
        self.t = []

    def callback(self, msg):

        global count 
        global start_time 
        
        if(count == 0):
            start_time = msg.time
        
        if(count == 1):
            self.speed.append(msg.filtered_encoder)
            self.time.append(msg.time - start_time)
            self.t.append(self.number)
            self.number = self.number + 1
            print(msg.time - start_time)

        count = 1

        if (self.number == 300):
            plt.plot(self.t, self.speed)
            plt.grid(True)
            plt.show()

        

if __name__ == '__main__':
    try:
        settings = termios.tcgetattr(sys.stdin)
        init()

    except rospy.ROSInterruptException:
        pass
