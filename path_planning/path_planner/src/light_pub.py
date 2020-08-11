#! /usr/bin/env python

import rospy
import time
import serial
import math
import sys, select, termios, tty
from std_msgs.msg import String
from std_msgs.msg import UInt32
from core_msgs.msg import Control

def init():
    
    pub_time = rospy.Publisher('light_state', UInt32, queue_size=10)
    rospy.init_node('light_pub', anonymous=True)
    rate = rospy.Rate(20)
    msg = UInt32()
    green_light = 1
    left_light = 2
    yellow_light = 4
    red_light = 8
    while not rospy.is_shutdown():
        print("1 : green light / 2 : red light / 3 : left light  \n/ 4 : green left light / 5 : red left light   ")
        key = getKey()
        print(msg)
        if key == '1':  #green light
            msg.data = green_light
        if key == '2':  #red light
            msg.data = red_light
        if key == '3':  #left light
            msg.data = left_light
        if key == '4':  #green left light
            msg.data = green_light + left_light
        if key == '5':  #red left light
            msg.data = red_light + left_light
        if key == 'q':
            rospy.on_shutdown()        
        pub_time.publish(msg)

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