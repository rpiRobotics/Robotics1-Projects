import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy
import string
import time
import numpy as np
from matplotlib import pyplot as plt
import math

MAX_FORWARD_VELOCITY = 0.65

#Should be 1.5 (pi/2) but doesn't seem to really reach more than 1.35
MAX_PHI = 1.5
D_FILTER_SIZE = 5
PHI_FILTER_SIZE = 5
D_AVERAGE_SIZE = 3
PHI_AVERAGE_SIZE = 3


def s():
    duck.sendCmd(0,0)

def getTrim(v):
    trim = 2.7 * v
    return trim

def getVf(phi):
    vf = - ((MAX_FORWARD_VELOCITY) / (MAX_PHI)) * abs(phi) + MAX_FORWARD_VELOCITY
    if (vf < 0.5):
        vf = 0.375
    return vf

dList = []
def getFilteredD(d):
    dList.append(d)
    if (len(dList) > D_FILTER_SIZE):
        dList.pop(0)
    return numpy.median(dList)

phiList = []
def getFilteredPhi(phi):
    phiList.append(phi)
    if (len(phiList) > PHI_FILTER_SIZE):
        phiList.pop(0)
    return numpy.median(phiList)
   
dAvgList = []
def getAvgD(d):
    dAvgList.append(d)
    if (len(dAvgList) > D_AVERAGE_SIZE):
        dAvgList.pop(0)
    return numpy.average(dAvgList)

phiAvgList = []
def getAvgPhi(phi):
    phiAvgList.append(phi)
    if (len(phiAvgList) > PHI_AVERAGE_SIZE):
        phiAvgList.pop(0)
    return numpy.average(phiAvgList)

if __name__ == '__main__':
    RRN.UseNumPy=True
    # Register a Local transport
    t1 = RR.LocalTransport()
    RRN.RegisterTransport(t1)

    # Register a TCP transpor
    t2 = RR.TcpTransport()
    RRN.RegisterTransport(t2)
    print("Connecting...")
    duck = RRN.ConnectService("tcp://10.13.215.108:2000/DuckiebotServer.duckieryan/Duckiebot")
    print("Connection Successful!")

    error = 0
    phi = 0
    ERROR_SUM = 0
    PHI_SUM = 0
    d0 = 0
    d1 = 0
    d2 = 0
    while(True):
        a = duck.lane_pose.d - 0.030700389
        b = duck.lane_pose.phi - 0.029922179
        a = -20. * a
        b = -7.5* b
        duck.sendCmd(0.6, a + b + getTrim(.6))
        print(" "*(20-len(str(a)))+str(a)+" "*(20-len(str(b)))+str(b)+" "*(20-len(str(a+b)))+str(a+b))

        
        error = getFilteredD(duck.lane_pose.d)
        phi = getFilteredPhi(duck.lane_pose.phi)

        error = getAvgD(error)
        phi = getAvgPhi(phi)
        
        ERROR_SUM = .8 * ERROR_SUM + 0.2 * error
        PHI_SUM = 0.5 * PHI_SUM + 0.5 * phi

        #phi = PHI_SUM
        
        p = -8 * error
        i = -1.5* ERROR_SUM
        d = -3.5 * phi

        d0 = error + phi + ERROR_SUM
        deriv = -2*(d2 - d0)
        d1 = d0
        d2 = d1
        
        
        w = p + i + d + deriv
        w = 2.3 * w
        
        vf = getVf(phi)
        trim = getTrim(vf+0.3)
        
        #print(error, phi)   
        #duck.sendCmd(vf, w + trim)
        #print(" "*(18-len(str(p)))+str(p)+" "*(18-len(str(i)))+str(i)+" "*(18-len(str(d)))+str(d)+" "*(18-len(str(deriv)))+str(deriv)) 

        
