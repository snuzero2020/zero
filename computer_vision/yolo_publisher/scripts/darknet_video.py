#!/usr/bin/env python3
from ctypes import *
import math
import random
import os
import cv2
import numpy as np
import time
import darknet
import rospy
from std_msgs.msg import Int32

def convertBack(x, y, w, h):
    xmin = int(round(x - (w / 2)))
    xmax = int(round(x + (w / 2)))
    ymin = int(round(y - (h / 2)))
    ymax = int(round(y + (h / 2)))
    return xmin, ymin, xmax, ymax


def cvDrawBoxes(detections, img):
    max_area = 0
    traffic_pub_data = 0
    for detection in detections:
        x, y, w, h = detection[2][0],\
            detection[2][1],\
            detection[2][2],\
            detection[2][3]
        if detection[0].decode() == "GREEN" or detection[0].decode() == "GREEN_LEFT" or detection[0].decode() == "RED" or detection[0].decode() == "RED_LEFT":
            if w*h > max_area:
                if detection[0].decode() == "GREEN":
                    traffic_pub_data = 1
                elif detection[0].decode() == "GREEN_LEFT":
                    traffic_pub_data = 3
                elif detection[0].decode() == "RED":
                    traffic_pub_data = 8
                elif detection[0].decode() == "RED_LEFT":
                    traffic_pub_data = 10
                max_area = w*h

            # else:
                xmin, ymin, xmax, ymax = convertBack(
                    float(x), float(y), float(w), float(h))
                pt1 = (xmin, ymin)
                pt2 = (xmax, ymax)
                cv2.rectangle(img, pt1, pt2, (0, 255, 0), 1)
                cv2.putText(img,
                            detection[0].decode() +
                            " [" + str(round(detection[1] * 100, 2)) + "]",
                            (pt1[0], pt1[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            [0, 255, 0], 2)
                            
    if not traffic_pub_data == 0:
        traffic_pub.publish(traffic_pub_data)
    return img


netMain = None
metaMain = None
altNames = None

def YOLO():

    global metaMain, netMain, altNames
    configPath = "/home/snuzero/catkin_ws/src/zero/computer_vision/yolo_publisher/scripts/yolo-obj.cfg"
    weightPath = "/home/snuzero/catkin_ws/src/zero/computer_vision/yolo_publisher/scripts/yolo-obj_last.weights"
    metaPath = "/home/snuzero/catkin_ws/src/zero/computer_vision/yolo_publisher/scripts/obj.data"
    if not os.path.exists(configPath):
        raise ValueError("Invalid config path `" +
                         os.path.abspath(configPath)+"`")
    if not os.path.exists(weightPath):
        raise ValueError("Invalid weight path `" +
                         os.path.abspath(weightPath)+"`")
    if not os.path.exists(metaPath):
        raise ValueError("Invalid data file path `" +
                         os.path.abspath(metaPath)+"`")
    if netMain is None:
        netMain = darknet.load_net_custom(configPath.encode(
            "ascii"), weightPath.encode("ascii"), 0, 1)  # batch size = 1
    if metaMain is None:
        metaMain = darknet.load_meta(metaPath.encode("ascii"))
    if altNames is None:
        try:
            with open(metaPath) as metaFH:
                metaContents = metaFH.read()
                import re
                match = re.search("names *= *(.*)$", metaContents,
                                  re.IGNORECASE | re.MULTILINE)
                if match:
                    result = match.group(1)
                else:
                    result = None
                try:
                    if os.path.exists(result):
                        with open(result) as namesFH:
                            namesList = namesFH.read().strip().split("\n")
                            altNames = [x.strip() for x in namesList]
                except TypeError:
                    pass
        except Exception:
            pass
    cap = cv2.VideoCapture(0)
    #cap = cv2.VideoCapture("/home/snuzero/darknet/tl10_test.avi")
    cap.set(3, 1920)
    cap.set(4, 1080)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
    os.system('v4l2-ctl -d 0 -c backlight_compensation=1')
    #day exposure
    #cap.set(cv2.CAP_PROP_EXPOSURE, 0.0010)
    #night exposure
    # cap.set(cv2.CAP_PROP_EXPOSURE, 0.0108)
    #rainy exposure
    cap.set(cv2.CAP_PROP_EXPOSURE, 0.0020)

    #day brightness
    #cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.3961)
    #night brightness
    #cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.5882)
    #rainy brightness
    cap.set(cv2.CAP_PROP_BRIGHTNESS, 0.2510)

    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
    #cap.set(32, 1)
    cap.set(cv2.CAP_PROP_GAIN, 0.2353)

    exposure = cap.get(cv2. CAP_PROP_EXPOSURE)
    brightness = cap.get(cv2. CAP_PROP_BRIGHTNESS)
    
    #width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    #height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    #backlight = cap.get(32)
    gain = cap.get(cv2.CAP_PROP_GAIN)

    print("Current Exposure:%0.4f, Current Brightness:%0.4f" % (exposure, brightness))
    #print("Width:%d, Height:%d" % (width, height))
    #print("Backlight Compensation:%d" % backlight)
    print("Gain:%0.4f" % gain)


    #out = cv2.VideoWriter(
    #    "out.mp4", cv2.VideoWriter_fourcc(*"MJPG"), 10.0,
    #    (darknet.network_width(netMain), darknet.network_height(netMain)))
    print("Starting the YOLO loop...")

    # Create an image we reuse for each detect
    darknet_image = darknet.make_image(darknet.network_width(netMain),
                                    darknet.network_height(netMain),3)
    while True:
        prev_time = time.time()
        ret, frame_read = cap.read()
        frame_rgb = cv2.cvtColor(frame_read, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb,
                                   (darknet.network_width(netMain),
                                    darknet.network_height(netMain)),
                                   interpolation=cv2.INTER_LINEAR)
        
        height, width, channel = frame_resized.shape
        matrix = cv2.getRotationMatrix2D((darknet.network_width(netMain)/2, darknet.network_height(netMain)/2), 90, 1)
        frame_resized = cv2.warpAffine(frame_resized, matrix, (darknet.network_height(netMain),darknet.network_width(netMain)))
        darknet.copy_image_from_bytes(darknet_image,frame_resized.tobytes())

        detections = darknet.detect_image(netMain, metaMain, darknet_image, thresh=0.85)
        image = cvDrawBoxes(detections, frame_resized)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        print(1/(time.time()-prev_time))
        cv2.imshow('Love_ZERO', image)
        key = cv2.waitKey(2)
        if key == 27:
            print("quit")
            break
    cap.release()
    #out.release()

if __name__ == "__main__":
    traffic_pub = rospy.Publisher('/light_state', Int32, queue_size= 10)
    rospy.init_node("traffic_light_publisher_video", anonymous= True)
    YOLO()
