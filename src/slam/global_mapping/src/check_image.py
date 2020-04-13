import rospy
import rosbag
import numpy as np
import sys
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
from struct import pack, unpack
import cv2
from cv_bridge import CvBridge


topic_name = '/camera/color/image_raw'
bag_file_path = 'check_rtab.bag'
bag = rosbag.Bag(bag_file_path, 'r')
times = []
topics = []
start_time = bag.get_start_time()
for topic, msg, time in bag.read_messages(topic_name):
    this_time = time.to_time()-start_time
    times.append(this_time)
    topics.append(msg)
    bridge = CvBridge()
    #cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    #cv2.imshow('img',cv_image)
    #cv2.waitKey(1)
    

img = topics[0] # sensor_msgs/image
cv_image = bridge.imgmsg_to_cv2(img,"bgr8")
cv2.imshow('img',cv_image)
cv2.waitKey(10000)
"""
data = []
for i in range(0, len(img.data)/3):
    data.append([uint8(img.data[3*i]), uint8(img.data[3*i+1]), uint8(img.data[3*i+2])])

print(data)
"""
"""
bridge = CvBridge()
cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
cv2.imshow('img',cv_image)
cv2.waitKey()
"""
bag.close()


