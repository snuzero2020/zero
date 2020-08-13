#!/usr/bin/env python3
from cv_bridge import CvBridge
import cv2
import torch
import numpy as np
import time
from sensor_msgs.msg import Image
from torchvision.transforms import transforms
import rospy

from model import *
from utils import *

video_path = "/home/snuzero/Pictures/vision_lane_data/lane_video/lane_detection_1/front_lane_detection_1/front_lane_detection_1.avi"
weight_path = "/home/snuzero/catkin_ws/src/zero/computer_vision/lanenet_lane_detection/ldln_ckpt_7.pth"

if __name__ == "__main__":
    
    USE_CUDA = torch.cuda.is_available()
    DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    model = LaneNet().to(DEVICE)
    model.load_state_dict(torch.load(weight_path))
    model.eval()

    bridge = CvBridge()

    seg_pub = rospy.Publisher('/lane_seg_topic', Image, queue_size = 10)
    rospy.init_node('LaneNet_publisher_node', anonymous = True)
    rate = rospy.Rate(100)

    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print("Video is not opened!")
    else:
        while not rospy.is_shutdown():
            ret, img = cap.read()

            if ret:
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                img = cv2.resize(img, (640, 480))
                img_to = transforms.ToTensor()(img)
                img_to = torch.stack([img_to]).to(DEVICE)

                start_time = time.time()
                output = model(img_to)
                pix_embedding = output['pix_embedding']
                pix_embedding = pix_embedding.detach().cpu().numpy()
                embedding = np.transpose(pix_embedding[0],(1,2,0))
                binary_seg = output['binary_seg']
                binary_seg_prob = binary_seg.detach().cpu().numpy()
                binary_seg_pred = np.argmax(binary_seg_prob, axis = 1)[0]
                img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                seg_img = np.zeros_like(img)

                seg_img[binary_seg_pred == 1] = (255, 255, 255)
                seg_message = bridge.cv2_to_imgmsg(seg_img, "bgr8")

                seg_pub.publish(seg_message)
                rate.sleep()
            else:
                break