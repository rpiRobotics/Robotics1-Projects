import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy
import string
import time
import numpy as np
from matplotlib import pyplot as plt

def stop():
    duck.sendStop()

def move(v, w, t):
    duck.sendCmd(v, w)
    time.sleep(t)
    stop()

def circle(t):
    duck.sendCmd(0,100)
    time.sleep(t)
    duck.sendCmd(0,-100)
    time.sleep(t*0.0000001)
    stop()

def s(v, t, s, t2, s2):
    duck.sendCmd(v, t)
    time.sleep(s)
    duck.sendCmd(v, t2)
    time.sleep(s2)
    stop()
    
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
