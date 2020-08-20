#! /usr/bin/env python

import rospy
import time
import serial
import math
import sys, select, termios, tty
from std_msgs.msg import String
from std_msgs.msg import Int32
from core_msgs.msg import Control

def init():
    
    pub_time = rospy.Publisher('parking_space', Int32, queue_size=10)
    rospy.init_node('parking_lot_pub', anonymous=True)
    rate = rospy.Rate(20)
    msg = Int32()
    while not rospy.is_shutdown():
        print("s : searching spot, 0 ~ 5 : parking spot")
        key = getKey()
        print(msg)
        if key == 's':  #searching spot
            msg.data = -1
        if key == '0':  #parking spot 0
            msg.data = 0
        if key == '1':  #parking spot 1
            msg.data = 1
        if key == '2':  #parking spot 2
            msg.data = 2
        if key == '3':  #parking spot 3
            msg.data = 3
        if key == '4':  #parking spot 4
            msg.data = 4
        if key == '5':  #parking spot 5
            msg.data = 5
        if key == '6':
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