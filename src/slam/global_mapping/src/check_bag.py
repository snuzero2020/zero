import rospy
import rosbag


topic_name = '/camera/color/image_raw'
bag_file_path = 'study_cafe.bag'
bag = rosbag.Bag(bag_file_path, 'r')
times = []
topics = []
start_time = bag.get_start_time()
for topic, msg, time in bag.read_messages(topic_name):
    times.append(time.to_time()-start_time)
    topics.append(msg)
    #print(msg)

"""
#image topic
print(len(topics))
print(type(topics[0]))
img = topics[0]
print(img.height)
print(img.width)
print(len(img.data))
print(img.encoding)
print(img.is_bigendian)
print(img.step)
"""
bag.close()