#!/usr/bin/env python
# license removed for brevity
import rospy
import time
from core_msgs.msg import VehicleState
from slam.msg import Data
from matplotlib import pyplot as plt
import sys, select, termios, tty
from std_msgs.msg import Float64

def init():
    while not rospy.is_shutdown():
        print("control_test")
        rospy.init_node('control_test', anonymous = True)
        vehiclestate = getVehicleState()
        #rospy.Subscriber("filtered_data", Data, vehiclestate.callback)
        rospy.Subscriber("/vehicle_state",VehicleState , vehiclestate.callback)
        rospy.spin()

count = 0
start_time = 0.0
class getVehicleState():
    def __init__(self):
        self.speed = []
        self.steer = []
        self.time = []
        self.number = 0

    def callback(self, msg):

        global count 
        global start_time 

        if(count == 0):
            start_time =  float(msg.header.stamp.secs) + msg.header.stamp.nsecs / 1000000000.0 
        
        if(count == 1):
            self.speed.append(msg.speed)
            self.time.append((float(msg.header.stamp.secs) + msg.header.stamp.nsecs / 1000000000.0) - start_time)
            self.number = self.number + 1
            print(msg.speed)

        count = 1

        if (self.number == 1000):
            plt.plot(self.time, self.speed)
            plt.grid(True)
            plt.show()

        

if __name__ == '__main__':
    try:
        settings = termios.tcgetattr(sys.stdin)
        init()

    except rospy.ROSInterruptException:
        pass
