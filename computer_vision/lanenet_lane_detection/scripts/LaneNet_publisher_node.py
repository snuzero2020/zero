#!/usr/bin/env python3
import cv2
import torch
import numpy as np
import rospy
import time
from std_msgs.msg import UInt16MultiArray
from torchvision.transforms import transforms
from lanenet_lane_detection.msg import lanenet_msg
from rospy.numpy_msg import numpy_msg

from model import *
from utils import *

video_left = "/home/snuzero/Pictures/vision_lane_data/lane_video/lane_detection_1/front_lane_detection_1/front_lane_detection_1.avi"
video_right = "/home/snuzero/Pictures/vision_lane_data/lane_video/lane_detection_2/front_lane_detection_2/front_lane_detection_2.avi"
weight_path = "/home/snuzero/catkin_ws/src/zero/computer_vision/lanenet_lane_detection/ldln_ckpt_7.pth"

if __name__ == "__main__":
    
    USE_CUDA = torch.cuda.is_available()
    DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    
    model = LaneNet().to(DEVICE)
    model.load_state_dict(torch.load(weight_path))
    model.eval()

    seg_pub = rospy.Publisher('/lane_seg_topic', numpy_msg(lanenet_msg), queue_size = 10)

    rospy.init_node('LaneNet_publisher_node', anonymous = True)
    #rate = rospy.Rate(100)

    cap_left = cv2.VideoCapture(video_left)
    cap_right = cv2.VideoCapture(video_right)

    if not cap_left.isOpened():
        print("Left video is not opened!")
    if not cap_right.isOpened():
        print("Right video is not opened!")

    else:
        while not rospy.is_shutdown():
            ret_left, img_left = cap_left.read()
            ret_right, img_right = cap_right.read()

            if ret_left and ret_right:
                start_time = time.time()
                
                img_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2RGB)
                img_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2RGB)
                
                img_left = cv2.resize(img_left, (640, 480))
                img_right = cv2.resize(img_right, (640, 480))
               
                img_input_left = transforms.ToTensor()(img_left)
                img_input_right = transforms.ToTensor()(img_right)

                img_input = torch.stack([img_input_left, img_input_right], dim = 0).to(DEVICE)

                output = model(img_input)

                lanenet_msg_pub = lanenet_msg()

                #pix_embedding = output['pix_embedding']
                #pix_embedding = pix_embedding.detach().cpu().numpy()
                
                #print(pix_embedding.shape)
                #embedding_left = np.transpose(pix_embedding[0],(1,2,0))
                #embedding_right = np.transpose(pix_embedding[1],(1,2,0))
                
                binary_seg = output['binary_seg']
                binary_seg_prob = binary_seg.detach().cpu().numpy()
                lanenet_msg_pub = binary_seg_prob.flatten()
                
                #binary_left_seg_pred = np.argmax(binary_seg_prob, axis = 1)[0]
                #binary_right_seg_pred = np.argmax(binary_seg_prob, axis = 1)[1]
                
                #img_left = cv2.cvtColor(img_left, cv2.COLOR_RGB2BGR)
                #img_right = cv2.cvtColor(img_right, cv2.COLOR_RGB2BGR)
                
                #left_seg_img = np.zeros_like(img_left)
                #right_seg_img = np.zeros_like(img_right)

                #left_seg_img[binary_left_seg_pred == 1] = (255, 255, 255)
                #right_seg_img[binary_right_seg_pred == 1] = (255, 255, 255)
                #seg_img = seg_img.tolist()
                #seg_msg = UInt16MultiArray()
                #seg_msg.data = seg_img

                seg_pub.publish(lanenet_msg_pub)
                #rate.sleep()
                #cv2.imshow("left_seg", left_seg_img)
                #cv2.imshow("right_seg", right_seg_img)
                #cv2.waitKey(1)
                print(time.time() - start_time)
            else:
                break