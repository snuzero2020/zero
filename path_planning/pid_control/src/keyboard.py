#!/usr/bin/env python
# license removed for brevity
# input : steer-degree, speed:m/s

import rospy
import time
import serial
import math
import sys, select, termios, tty
from std_msgs.msg import String
from std_msgs.msg import Float64
from core_msgs.msg import Control

def init():
    
    pub_time = rospy.Publisher('start_time', Float64, queue_size=10)
    rospy.init_node('keyboard_input', anonymous=True)
    rate = rospy.Rate(20)
    msg = Control()
    is_auto=1
    estop=0
    gear=0
    speed=0.0 #m/s
    steer=0.0 #degree
    brake=0
    time = rospy.get_param('/time', 0) + 0.3 #seconds
    init = rospy.Time.now()

    while not rospy.is_shutdown():
        cur = rospy.Time.now()
        if time!=0 :
            t = cur-init
        key = getKey()
        print("press the key" + key)

        if key == 'c':
            time = Float64()
            time.data = rospy.get_time()
            pub_time.publish(time)
        

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
