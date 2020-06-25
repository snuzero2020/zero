import rospy
import rosbag
import numpy as np
import sys

topic_name = '/voxel_cloud'
bag_file_path = 'cafe_voxel.bag'
bag = rosbag.Bag(bag_file_path, 'r')
times = []
topics = []
start_time = bag.get_start_time()
for topic, msg, time in bag.read_messages(topic_name):
    this_time = time.to_time()-start_time
    times.append(this_time)
    topics.append(msg)
    #print(msg)

print(len(topics))
sample = topics[0]
print(sample.fields)
print(sample.row_step)
print(sample.point_step)
print(sample.height)
print(sample.width)
print(sample.row_step * sample.height)
print(sample.is_dense)
print(len(sample.data))
for i in range(0,32):
    print(np.fromstring(sample.data[i], dtype=np.uint8))
    if i%4 == 3:
        print("")
bag.close()