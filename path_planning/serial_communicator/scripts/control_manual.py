#!/usr/bin/env python
# license removed for brevity
# input : steer-degree, speed:m/s

import rospy
import time
import serial
import math
import sys, select, termios, tty
from std_msgs.msg import String
from core_msgs.msg import Control

def init():
    
    pub = rospy.Publisher('/car_signal', Control, queue_size=10)
    rospy.init_node('keyboard_input', anonymous=True)
    rate = rospy.Rate(20)
    msg = Control()
    is_auto=1
    estop=0
    gear=0
    speed=rospy.get_param('/speed', 0) #m/s
    steer=rospy.get_param('/steer', 0) #degree
    brake=1
    time = rospy.get_param('/time', 0) + 0.3 #seconds
    init = rospy.Time.now()
    while not rospy.is_shutdown():
        cur = rospy.Time.now()
        if time!=0 :
            t = cur-init
        key = getKey()
        print("key" + key)
        
        if key == 'q':
            break
        elif key == 'a':
            steer = steer - 1
            if steer <= -15 :
                steer = -15
        elif key == 'd':
            steer = steer + 1
            if steer >= 15 :
                steer = 15
        elif key == 'w':
            speed = speed + 0.1
            if speed >= 3:
                speed = 3
        elif key == 's':
            speed = speed - 0.1
            if speed < -3:
                speed = -3
        elif key == 'e':
            if gear == 0:
                gear = 2
            elif gear == 2:
                gear = 0
        elif key == 'b':
            if brake == 1:
                brake = 50
            elif brake == 50:
                brake = 1
         
        rospy.loginfo(msg)
        
        msg.is_auto = is_auto
        msg.estop = estop
        msg.gear = gear
        msg.brake = brake
        msg.speed = round(speed,3)
        msg.steer = round(steer,3)
        msg.header.stamp = rospy.Time.now()
        
        pub.publish(msg)
        rate.sleep()
        
def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == '__main__':
    try:
        settings = termios.tcgetattr(sys.stdin)
        init()
    except rospy.ROSInterruptException:
        pass
