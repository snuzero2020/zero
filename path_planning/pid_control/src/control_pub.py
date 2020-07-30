#!/usr/bin/env python
# license removed for brevity
import rospy
import time
import random
from core_msgs.msg import VehicleState

def init():
    pub = rospy.Publisher('/vehicle_state', VehicleState, queue_size = 10)
    rospy.init_node('control_pub', anonymous = True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = VehicleState()
    
        msg.is_auto = 0
        msg.estop = 0
        msg.gear = 0
        msg.brake = 0
        msg.speed = random.random()
        msg.steer = 0
        msg.encoder = 0
        msg.alive = 0
        msg.header.stamp = rospy.Time.now()

        pub.publish(msg)     

        print (msg.speed) 
        rate.sleep()  
        


if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass 
