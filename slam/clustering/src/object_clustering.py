import rospy
import rosbag
import numpy as np
import sys
import ros_numpy
import sensor_msgs.point_cloud2 as pc2
from struct import pack, unpack
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import random
import math
from sklearn.neighbors import KDTree

def clusterHelper(index, points, cluster, processed, tree, distanceTol):
    processed[index] = True
    cluster.append(index)
    nearest_index, nearest_distance = tree.query_radius([points[index]], r=distanceTol, return_distance=True)
    for i in range(len(nearest_index[0])):
        idx = nearest_index[0][i]
	if processed[idx] == False:
	    clusterHelper(idx, points, cluster, processed, tree, distanceTol)



def euclideanCluster(points, tree, distanceTol):
    clusters = []
    processed = [False for i in range(len(points))]
    i = 0
    while i < len(points):
    	if processed[i] == True:
            i += 1
	    continue
	cluster = []
	clusterHelper(i, points, cluster, processed, tree, distanceTol)
	if(len(cluster) > 10):
            print(len(cluster))
	    clusters.append(cluster)
            cluster = []
	    i += 1
    return clusters



def RansacPlane(points, maxIterations, distanceTol):
    # Initialize unordered set inliersResult
    inliersResult = {}
    inliersResult = set()
    result = {'a':0.0, 'b':0.0, 'c':0.0, 'd':0.0}
    while maxIterations:
        # Initialize unordered set inliers
        inliers = {}
        inliers = set()
        # Pick 3 Random Samples
        while len(inliers) < 3 :
            inliers.add(random.randint(0,len(points)-1))
        inliers_iter = inliers.__iter__()
        itr = next(inliers_iter)
        x1 = points[itr][0]
        y1 = points[itr][1]
        z1 = points[itr][2]
        itr = next(inliers_iter)
        x2 = points[itr][0]
        y2 = points[itr][1]
        z2 = points[itr][2]
        itr = next(inliers_iter)
        x3 = points[itr][0]
        y3 = points[itr][1]
        z3 = points[itr][2]
        # Get Normal Vector by Cross Product
        a = ((y2-y1)*(z3-z1) - (y3-y1)*(z2-z1))
        b = ((z2-z1)*(x3-x1) - (x2-x1)*(z3-z1))
        c = ((x2-x1)*(y3-y1) - (x3-x1)*(y2-y1))
        d = -(a*x1 + b*y1 + c*z1)
        for i in range(len(points)):
            # Not consider three points already picked
            if i in inliers:
                continue
            x4 = points[i][0]
            y4 = points[i][1]
            z4 = points[i][2]
            # Distance between picked point and the plane
            dist = math.fabs(a*x4 + b*y4 + c*z4 +d) / math.sqrt(a*a + b*b + c*c)
            if dist <= distanceTol:
                inliers.add(i)
        if len(inliers) > len(inliersResult):
            inliersResult = inliers
            result['a'] = a
            result['b'] = b
            result['c'] = c
            result['d'] = d
        maxIterations -= 1
    return inliersResult, result


topic_name = '/velodyne_points'
bag_file_path = 'change_lane.bag'
bag = rosbag.Bag(bag_file_path, 'r')
times = []
topics = []
start_time = bag.get_start_time()
for topic, msg, time in bag.read_messages(topic_name):
    this_time = time.to_time()-start_time
    times.append(this_time)
    topics.append(msg)
    #print(msg.fields)
    #print(len(msg.data))
    #print(msg)
X=[]
Y=[]
Z=[]
channels=[]
projected_X=[]
projected_Y=[]
projected_points = []
points = []

cnt = 0
for point in pc2.read_points(topics[133],field_names={"x","y","z","intensity","ring"},skip_nans=True):
    #print(cnt)
    #print(point[0], point[1], point[2],point[3],point[4]) #r,g,b
    if point[0] >0 and (point[0]*point[0]+point[1]*point[1]+point[2]*point[2])<25:
        cnt =  cnt +1
        points.append([point[0],point[1],point[2]])
        channels.append(point[4])

inlier, plane = RansacPlane(points, 100, 0.05)
for i in range(len(points)):
    if i not in inlier:
        X.append(points[i][0])
        Y.append(points[i][1])
        Z.append(points[i][2])

a = plane['a']
b = plane['b']
c = plane['c']
d = plane['d']
for i in range(len(X)):
    p,q,r = X[i], Y[i], Z[i]
    t = (p*a + q*b + r*c + d)/(a*a+b*b+c*c)
    projected_X.append(p-a*t)
    projected_Y.append(q-b*t)
    projected_points.append([p-a*t,q-b*t])
projected_points = np.array(projected_points)
tree = KDTree(projected_points)
clusters = euclideanCluster(projected_points, tree, 0.1)

print(clusters)
C=[0]*len(X)
i = 0
for cluster in clusters:
    i = i+1
    for index in cluster:
        C[index] = i
print(i)
fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(projected_X, projected_Y, c=C, s=0.2)
plt.show()
"""
fig = plt.figure()
ax = fig.add_subplot(111)
ax.scatter(X,Y, s=0.5)
plt.show()
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(X,Y,Z, s=0.5)
plt.show()
"""
"""
fig=plt.figure()
ax = fig.add_subplot(111)
ax.scatter(X,Y,s=0.2,c=C/255.0)
plt.show()
"""
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
