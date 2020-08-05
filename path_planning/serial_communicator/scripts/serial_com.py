#!/usr/bin/env python
# license removed for brevity
# edit example

import rospy
import time
import serial
import math
import os
from std_msgs.msg import String
from core_msgs.msg import VehicleState
from core_msgs.msg import Control

alive = 0
enc = []

sudoPassword = 'snuzero123'
command = 'chmod 777 /dev/ttyUSB2'
p= os.system('echo %s|sudo -S %s' % (sudoPassword, command))



def init():

    pub = rospy.Publisher('/vehicle_state', VehicleState, queue_size=10)
    control_data = getControlData()
    rospy.Subscriber("/calibrated_control", Control, control_data.callback)
    rospy.init_node('serial_com', anonymous=True)
    rate = rospy.Rate(50)
    msg = VehicleState() #define msg - current platform state

    while not rospy.is_shutdown():
        with serial.Serial(port='/dev/ttyUSB2',
                           baudrate=115200,
                           parity=serial.PARITY_NONE,
                           stopbits=serial.STOPBITS_ONE,
                           bytesize=serial.EIGHTBITS,
                           timeout=1) as ser:

            while not rospy.is_shutdown():
                msg=MsgUpdate(msg,ser)
                rospy.loginfo(msg)
                pub.publish(msg)

                sendSerial(ser,control_data)
                rate.sleep()



def MsgUpdate(msg,ser): #message about current sate (serial data : platform->upper)

    raw_data = ser.read(18)
    data = []
    for i in range(0,18):
        data.append(hex(int(raw_data[i].encode('hex'),16)))
    is_auto = int(data[3],16)
    estop = int(data[4],16)
    if(estop == 16):
        estop = 1
    gear = int(data[5],16)


    #Steer calculation # degree
    if(len(data[9])>=4):
        steer = int(data[9],16) * 256 + int(data[8],16) + 1 - pow(2,15)
        steer = -float(steer)/71 +462
    else:
        steer = int(data[9],16) * 256 + int(data[8],16)
        steer = -float(steer)/71


    brake = int(data[10],16)

    global alive
    alive = int(data[15],16)

    #encoder caculation

    encoder = int(data[14],16) * pow(256,3) + int(data[13],16) * pow(256,2) + int(data[12],16) * pow(256,1) + int(data[11],16)
    if encoder > pow(256,4) * 0.75: # in case encoder < 0
        encoder = encoder - pow(256,4)

    #Speed caculation : m/s
    global enc
    radius = 0.266 # meter scale
    distance = 2 * math.pi * radius # distance per rotation
    enc.append(encoder)
    if(len(enc)>=20):
        speed = ((enc[19] - enc[0]) / float(100)) * float(distance) # m/s
        enc.pop(0)
    else:
        speed = 0 #initial speed : 0 m/s

    if not isValidValue(speed, steer): return msg #in case of invalid speed or steer -> no update of msg

    #setting message variables
    msg.is_auto = is_auto
    msg.estop = estop
    msg.gear = gear
    msg.brake = brake
    msg.speed = round(speed,3)
    msg.steer = -round(steer,3)
    print("current : ", msg.steer, '\n')
    #msg.steer = round(steer+1.5,3)
    msg.encoder = encoder
    msg.alive = alive
    msg.header.stamp = rospy.Time.now()
    return msg

def sendSerial(ser,data): #upper->platform
    is_auto=data.is_auto
    estop=data.estop
    gear=data.gear
    speed=data.speed
    steer1=data.steer1
    steer2=data.steer2
    brake=data.brake
    global alive
    #rospy.loginfo("speed " + str(speed))
    #rospy.loginfo("steer1 " + str(steer1))
    #rospy.loginfo("steer2 " + str(steer2))
    #rospy.loginfo("brake " + str(brake))
    #rospy.loginfo("alive " + str(alive))
    data_array = bytearray([83, 84, 88, is_auto, estop, gear, 0, speed, steer1, steer2, brake, alive, 13, 10])
    ser.write(data_array)

def isValidValue(speed, steer): #speed : m/s, steer : degree
    if abs(speed) <= 10 and abs(steer) <= 35:
        return True
    else:
        return False

class getControlData(): #input:speed(m/s), steer(degree) -> output: speed(km/h * 10), steer(degree*71, steer1:first byte, steer2:second byte)
    def __init__(self):
        self.is_auto = 1
        self.estop = 0
        self.gear = 0
        self.speed = 0
        self.steer1 = 0
        self.steer2 = 0
        self.brake=200
        pass

    def callback(self,data_): #serial data update (upper->platform)

        print("callback")
        self.is_auto = data_.is_auto
        
        self.estop = data_.estop
        self.gear = data_.gear
        self.speed = int(float(data_.speed) * 3600 / 1000 * 10) # (m/s -> km/h * 10)
        # rad to degree
        print("data_.steer : ~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print(data_.steer)
	steer_degree = data_.steer
        print("steer_Degree : ~~~~~~~~~~~~~~~~~~~~~~~~~~~~")
        print(steer_degree)
        if steer_degree >=0:
            steer = int(float(steer_degree)*71)
            steer_low = steer%256
            steer_high = (steer-steer_low)/256
        else:
            steer = pow(2,15) + int(steer_degree*71)
            steer_low = steer%256
            steer_high = (steer-steer_low)/256 + pow(2,7)
	if (steer_high >= 256):
	    steer_high = 255
	if (steer_low >= 256):
	    steer_low = 255        

	self.steer1 = steer_high
        self.steer2 = steer_low

        self.brake = data_.brake



if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
