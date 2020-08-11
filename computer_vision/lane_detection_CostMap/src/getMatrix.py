import numpy as np
# x, y, z from 
# roll = x-dir , pitch = y-dir , yaw = z-dir
def getExternalMatrix(x, y, z, roll, pitch, yaw):
    list_trans = [1, 0, 0, x, 0, 1, 0, y, 0, 0, 1, z, 0, 0, 0, 1]
    mat_trans = np.array(list_trans).reshape(4,4)
    Rotx = np.array([[1,0,0,0],[0,np.cos(roll),-np.sin(roll),0],[0,np.sin(roll),np.cos(roll),0],[0,0,0,1]])
    Roty = np.array([[np.cos(pitch),0,np.sin(pitch),0],[0,1,0,0],[-np.sin(pitch),0,np.cos(roll),0],[0,0,0,1]])
    Rotz = np.array([[np.cos(yaw),-np.sin(yaw),0,0],[np.sin(yaw),np.cos(yaw),0,0],[0,0,1,0],[0,0,0,1]])
    Ext = np.dot(np.dot(Rotx, Roty),np.dot(Rotz, mat_trans))
    print(Ext)
    return Ext

def getInternalMatrix

getExternalMatrix(1, 2, 3, np.pi, np.pi, np.pi)