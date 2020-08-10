#!/usr/bin/env python
import rospy
import numpy as np

if __name__ == '__main__':
    rospy.init_node('transform_finder')
    
    # define numpy arrays filled like [[x1,y1],[x2,y2],[x3,y3],...]
    B = np.array( [[298530.466936, 4137867.325115], [298535.837418, 4137863.026706],
                    [298524.417827, 4137871.901486], [298519.033455, 4137876.154827], 
                    [298471.385152, 4137731.262642], [298464.490417, 4137722.657086],
                    [298374.415699, 4137686.051617], [298364.962923, 4137673.999805],
                    [298276.160882, 4137579.208914], [298310.081536, 4137552.529006]
                    ], dtype ='f8') # coordinates before transform

    A = np.array( [[10218, 1543], [10406, 1680], 
                    [10018, 1393], [9834, 1254], 
                    [8349, 6114], [8127, 6416],
                    [5148, 7692], [4843, 8097],
                    [1932, 11332], [3086, 12192]
                    ], dtype ='f8') # coordinates after transform

    #A = np.array( [[11839, 1958], [12026,2103], 
    #                [11640, 1806], [11458, 1661], 
    #                [9881, 6490], [9652, 6790],
    #                [6652, 8006], [6338, 8407],
    #                [3367, 11584], [4504, 12460]
    #                ], dtype ='f8') # coordinates after transform
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
    print(bc)
    print(ac)
    print("error : ")

    affine_matrix = np.concatenate((c*R,p),axis=1)
    B1 = np.concatenate((B,np.ones((B.shape[0],1))),axis=1)
    A_from_B = np.dot(B1, affine_matrix.transpose())
    err = A_from_B - A
    print(err)
    print(np.sqrt(np.mean(err**2)))