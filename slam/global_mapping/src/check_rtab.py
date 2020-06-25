import rospy
import rosbag
import numpy as np
import sys
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
from struct import pack, unpack
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

topic_name = '/rtabmap/cloud_map'
bag_file_path = 'check_rtab.bag'
bag = rosbag.Bag(bag_file_path, 'r')
times = []
topics = []
start_time = bag.get_start_time()
for topic, msg, time in bag.read_messages(topic_name):
    this_time = time.to_time()-start_time
    times.append(this_time)
    topics.append(msg)
    #print(len(msg.data))
    #print(msg)

last = len(topics)
X=[]
Y=[]
Z=[]
C=[]
for point in pc2.read_points(topics[0],field_names={"x","y","z","rgb"},skip_nans=True):
    #print(cnt)
    rgb = pack('f', point[3])
    rgb = unpack('i', rgb)[0]
    r = (rgb>>16)&0x0000ff
    g = (rgb>>8) & 0x0000ff
    b = (rgb) & 0x0000ff
    print(point[0], point[1], point[2],(rgb>>16)&0x0000ff, (rgb>>8) & 0x0000ff, (rgb) & 0x0000ff) #r,g,b
    if ((-0.5<point[2]) and (point[2] <1)):
        X.append(point[0])
        Y.append(point[1])
        Z.append(point[2])
        C.append([ r,g,b])
print(len(topics[0].data)/32)
C = np.array(C)
"""
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(X,Y,Z,c=C/255.0, s=0.1)
plt.show()
"""
fig=plt.figure()
ax = fig.add_subplot(111)
ax.scatter(X,Y,s=0.2,c=C/255.0)
plt.show()
"""
xyz_array = ros_numpy.point_cloud2.pointcloud2_to_array(topics[0])
for xyz in xyz_array:
    print(xyz[3])
"""
"""
points_list = []
for data in pc2.read_points(topics[0], skip_nans=True):
    points_list.append([data[0],data[1],data[2],data[3]])

pcl_data = pcl.PointCloud_PointXYZRGB()
pcl_data.from_list(points_list)
print(pcl_data)
bag.close()
"""