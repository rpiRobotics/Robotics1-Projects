import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy
import time

def stop():
    duck.sendStop()

def move(v, w, t):
    duck.sendCmd(v, w)
    time.sleep(t)
    stop()
    
def spin(trim, w, t):
    duck.sendCmd(0, w+trim)
    time.sleep(t)
    stop()
    
def circle(t):
    duck.sendCmd(0,100)
    time.sleep(t)
    duck.sendCmd(0,-100)
    time.sleep(t*0.0000001)
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

    #print(duck.lane_pose.phi)

    v = 0.5
    w = 1.0
    gain = 1
    #Trim without camera lag
    trim = 1.6
    #Trim with camera lag
    ctrim = 1.0
    d = 4.5
    T = d/(v*gain)
    
    for t in range(0,int(T*2),1):
        move(v*gain,w*trim,0.25)
        move(v*gain,w*trim*-1,0.25)
        trim = trim*0.85
    
    #circle(1.1)
    #print(duck.lane_pose.phi)
