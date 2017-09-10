# Determine the transformation between the end effector and arm-mounted camera
import numpy as np
import numpy.random as rng

import math3D
import DobotModel

def get_pose(angle_list,pca_list):
    """
    Input:
        angle_list - list of sets of joint angles [(a0,a1,a2)]
        pca_list - list of measured camera vectors [[3 x 1]]
    Output: (ptc,Rtc)
        ptc - estimated offset of the camera origin (c) from the end effector (t)
        Rtc - estimated rotation between the camera frame (c) and the end effector frame (t)
    """
    I = np.eye(3)
    b = []
    A = []
    # Form a large matrix of measurements
    for (angles,pca) in zip(angle_list,pca_list):
        Rt0 = np.transpose(DobotModel.R0T(angles))
        p0t = np.transpose(np.matrix(DobotModel.forward_kinematics(angles)))
        b.append(-Rt0*p0t)
        A.append(np.hstack((I,np.kron(I,np.transpose(pca)),-Rt0)))
    b = np.vstack(b)
    A = np.vstack(A)
    # Least-squares estimate (psuedo-inverse to solve for x)
    x = np.linalg.pinv(A)*b
    ptc = x[0:3]
    Rtc = nearest_rot(np.reshape(x[3:12],[3,3])) # enforce SO(3) properties
    p0a = x[12:15]
    return (ptc,Rtc,p0a)

def nearest_rot(M):
    """
    Input: a 3x3 matrix M
    Output: the rotation matrix nearest to M
    """
    (U,_,V) = np.linalg.svd(M)
    if np.linalg.det(U*V) < 0:
        return U*np.diag([1,1,-1])*V
    else:
        return U*V

def test():
    n = 20 # number of simulated measurements

    # Hidden parameters
    p0a = np.array([[215.0],[0],[-20]]) # mm
    ptc = np.array([[25.0],[10],[-30]]) # mm
    Rtc = math3D.rot([1.0,0.1,0.2],180) # deg

    # Simulate measurements
    angle_list = []
    pca_list = []
    for k in range(n):
        angles = (rng.normal(0,45),60*rng.rand(),60*rng.rand())
        # Dobot kinematics
        Rt0 = np.transpose(DobotModel.R0T(angles))
        p0t = np.transpose(np.matrix(DobotModel.forward_kinematics(angles)))
        # calculate camera vector
        pca = np.transpose(Rtc)*(Rt0*(p0a - p0t) - ptc)
        pca = pca + rng.normal(0,0.5,[3,1]) # add noise (mm)
        # add to lists
        angle_list.append(angles)
        pca_list.append(pca)

    # Estimate transformation
    (ptc_est,Rtc_est,p0a_est) = get_pose(angle_list,pca_list)
    ptc_est[2] = ptc_est[2] + p0a[2] - p0a_est[2] # could use all elements if confident about p0a
    print "ptc"
    print ptc
    print "ptc (estimate)"
    print ptc_est
    print "Rtc"
    print Rtc
    print "Rtc (estimate)"
    print Rtc_est
