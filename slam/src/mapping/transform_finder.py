#!/usr/bin/env python
import rospy
import numpy as np

if __name__ == '__main__':
    rospy.init_node('transform_finder')
    
    # define numpy arrays filled like [[x1,y1],[x2,y2],[x3,y3],...]
    B = np.array( [[0,0],[1,0],[1,1],[0,1]], dtype ='f8') # coordinates before transform
    A = np.array( [[1,0],[0,1],[1,2],[2,1]], dtype ='f8') # coordinates after transform
    print(B)
    print("to")
    print(A)

    bc = np.mean(B, axis=0, keepdims=True)
    Br = B-bc
    ac = np.mean(A, axis=0, keepdims=True)
    Ar = A-ac

    H = np.dot(Br.transpose(), Ar)

    u, s, vh = np.linalg.svd(H)
    R = np.dot(vh.transpose(), u.transpose())
    c = np.sum(Ar**2) / np.sum(s)
    p = -c*np.dot(R,bc.transpose())+ac.transpose()

    print("a1 : ", c*R[0,0], ", b1 : ", c*R[0,1], ", c1 : ", p[0,0])
    print("a2 : ", c*R[1,0], ", b2 : ", c*R[1,1], ", c2 : ", p[1,0])
    print("error : ")

    affine_matrix = np.concatenate((c*R,p),axis=1)
    B1 = np.concatenate((B,np.ones((B.shape[0],1))),axis=1)
    A_from_B = np.dot(B1, affine_matrix.transpose())
    err = A_from_B - A
    print(err)
    print(np.sqrt(np.mean(err**2)))