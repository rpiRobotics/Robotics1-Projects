import numpy as np

# Trigonometic function in degrees
def sind(th):
    return np.sin(np.deg2rad(th))

def cosd(th):
    return np.cos(np.deg2rad(th))

def tand(th):
    return np.tan(np.deg2rad(th))

def asind(x):
    return np.rad2deg(np.arcsin(x))

def acosd(x):
    return np.rad2deg(np.arccos(x))

def atan2d(x,y):
    return np.rad2deg(np.arctan2(x,y))

# Generate rotation matrices
def rotz(th):
    c,s = cosd(th),sind(th)
    return np.matrix([[c,-s,0],[s,c,0],[0,0,1]])

def roty(th):
    c,s = cosd(th),sind(th)
    return np.matrix([[c,0,s],[0,1,0],[-s,0,c]])

def rotx(th):
    c,s = cosd(th),sind(th)
    return np.matrix([[1,0,0],[0,c,-s],[0,s,c]])

def rot(k,th):
    k = k/np.linalg.norm(k)
    return np.eye(3) + sind(th)*hat(k) + (1-cosd(th))*hat(k)*hat(k)

# Matrix representing cross(k, )
def hat(k):
    return np.matrix([[0,-k[2],k[1]], [k[2],0,-k[0]], [-k[1],k[0],0]])
