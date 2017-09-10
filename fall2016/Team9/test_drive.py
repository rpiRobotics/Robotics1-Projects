#!/usr/bin/env python
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy
import string
import time
import pygame

pygame.mixer.init()
pygame.mixer.music.load("quack.mp3")

global d
global phi
global vel
global omega
global start_d
global start_phi
global trim
global april_tags
trim = 0.01

def TurnRight():
    stime = time.time();
    while(time.time() - stime < 1.5):
        duck.sendCmd(vel,-turningtrim*omega)
    duck.sendCmd(0,0)
    pauseDuck(1)

def TurnLeft():
    stime = time.time();
    x = float((1.57 - currenttheta)/0.25)
    while(time.time() - stime < 1):
        duck.sendCmd(0,turningtrim*omega)
    duck.sendCmd(0,0)
    pauseDuck(1)

def ForwardTo(vel,final):
    t = float(final/vel)
    stime = time.time()
    while(time.time() - stime < t):
        duck.sendCmd(vel,trim*omega)
    #duck.sendCmd(0,0)
    #pauseDuck(1)

def BackwardTo(vel,final):
    t = float(final/vel)
    stime = time.time()
    while(time.time() - stime < t):
        duck.sendCmd(-vel,-trim*omega)
    duck.sendCmd(0,0)
    pauseDuck(1)

def SquareLeft(vel,side):
    ForwardTo(vel,side)
    TurnLeft()
    ForwardTo(vel,side)
    TurnLeft()
    ForwardTo(vel,side)
    TurnLeft()
    ForwardTo(vel,side)
    TurnLeft()

def SquareRight(vel,side):
    ForwardTo(vel,side)
    TurnRight()
    ForwardTo(vel,side)
    TurnRight()
    ForwardTo(vel,side)
    TurnRight()
    ForwardTo(vel,side)
    TurnRight()

def printDuckposition():
    currentx = duck.x - x
    currenty = duck.y - y
    currenttheta = duck.theta - theta
    print ("X: ", currentx, " Y: ", currenty, " Theta: ", currenttheta)

def pauseDuck(t):
    stime = time.time()
    while(time.time()-stime < t):
        duck.sendCmd(0,0)

def getLanePose():
    lane_pose = duck.lane_pose
    d = lane_pose.d - start_d
    phi = lane_pose.phi -  start_phi
    print"Lane Pose Retrieved"
    print ("D: ", d, " Phi: ", phi)


def LaneFollow():
    global d
    global trim
    april_tags = duck.april_tags
    #print ("AR tags: ", len(april_tags))
    drive = True
    while drive :
        ForwardTo(vel, 0.0000001)
        getLanePose()
        if phi < 0:
            trim -= (turningtrim*(phi - start_phi)) - 0.05
            #trim -= 0.00001
        elif phi > 0:
            trim  += (turningtrim*(phi - start_phi)) - 0.1
            #trim += 0.00001
        april_tags = duck.april_tags
        print ("AR tags: ", len(april_tags))
        if len(april_tags) > 0:
            tag = april_tags[0]
            print tag.pos[2]
            if tag.pos[2] < 1:
                drive = False
                duck.sendCmd(0,0)
    duck.sendCmd(0,0)


def MoveYahCah():
    global april_tags
    drive = True
    count = 0
    quack = False

    while drive:
        ForwardTo(vel, 0.00000000001)
        april_tags = duck.april_tags
        #print ("AR tags: ", len(april_tags))
        if len(april_tags) > 0:
            tag = april_tags[0]
            print tag.pos[2]
            if tag.pos[2] < 0.975:
                drive = False
                duck.sendCmd(0,0)
                quack = True
                d = tag.pos[2]
    while quack:
        april_tags = duck.april_tags
        if len(april_tags) == 0:
            quack = False
            break
        else:
            tag = april_tags[0]
            print tag.pos[2]
            if tag.pos[2] - 0.1 > d or tag.pos[2] + 0.1 < d:
                quack = False
                break
            else:
                quack = True
                if count == 3:
                    #print "MOVE YAH QUACKING CAH!"
                    pygame.mixer.music.play()
                    pauseDuck(1.5)
                    count = 0
                else:
                    #print "Quack."
                    pygame.mixer.music.play()
                    pauseDuck(1.5)
                    count += 1  
                   
    duck.sendCmd(0,0)

if __name__ == '__main__':
    RRN.UseNumPy=True
    # Register a Local transport
    t1 = RR.LocalTransport()
    RRN.RegisterTransport(t1)

    # Register a TCP transpor
    t2 = RR.TcpTransport()
    RRN.RegisterTransport(t2)
    print "Connecting..."
    duck = RRN.ConnectService("tcp://quackincah.local:1234/DuckiebotServer.quackincah/Duckiebot")
    print "Connection Successful!"
    duck.sendCmd(0,0)

    vel = duck.v
    omega = duck.omega
    x = duck.x
    y = duck.y
    theta = duck.theta
    omega = 2
    vel = 0.5 #Open loop velocity
    vel = 0.025 #Closed loop velocity
    vel = 0.025
    currentx = duck.x
    currenty = duck.y
    currenttheta = duck.theta
    #trim = 0.017742
    #trim = -0.0006175
    #trim = -0.000225
    
    #trim = 0.01
    turningtrim = 0.75
    turningtrim = 1
    turningtrim = 0.000025
    trim = 0.0025

    start_lane_pose = duck.lane_pose
    start_d = start_lane_pose.d
    start_phi = start_lane_pose.phi
    print"Lane Pose Retrieved"
    print ("D: ", start_d, " Phi: ", start_phi)
    d = start_lane_pose.d
    phi = start_lane_pose.phi
    printDuckposition()

    MoveYahCah()
    #TurnLeft()
    #LaneFollow()
    
    #Open Loop Implementation
    """
    vel = 0.5
    omega = 2
    trim = 0.017742
    ForwardTo(0.5,1.275)
    """

    #Closed Loop Implementation
    """
    vel = 0.025
    omega = 2
    turningtrim = 0.000025
    start_lane_pose = duck.lane_pose
    start_d = start_lane_pose.d
    start_phi = start_lane_pose.phi
    LaneFollow()
    """

    #Special Skill Implementation
    vel = 0.025
    omega = 2
    trim = 0.017742
    MoveYahCah()

    #pauseDuck(1)
    #getLanePose()
    #printDuckposition()


    """
    printDuckposition()
    ForwardTo(vel,0.50)
    TurnRight()
    #SquareLeft(vel,0.608)
    #TurnLeft()
    printDuckposition()
    #TurnLeft()
    #SquareLeft(0.304,0.608)
    """
    

    # some processing required to get the image back to normal shape
    # openCV or numpy can probably do this pretty easy...
    """
    for i in (0,10):
        april_tags = duck.april_tags
    if len(april_tags) > 0:
        print "April Tag Retrieved"
        tag = april_tags[0]
        pos = tag.pos
        posx = pos[0]
        posy = pos[1]
        posz = pos[2]

        quat = tag.quat
        quatx = quat[0]
        quaty = quat[1]
        quatz = quat[2]
        quatw = quat[3]
    
        for i in (0, len(april_tags)-1):
            print april_tags[i]
            for i in (0, len(pos)-1):
                print (i, pos[i])
    """
    
    """
    vel = 0.127
    t = 1
    stime = time.time()
    print ("Start X: ", duck.x, " Start Y: ", duck.y)
    while(time.time()-stime < float(t)):
        duck.sendCmd(float(vel),-0.017675*omega)
    print ("X: ", duck.x - x, " Y: ", duck.y -y)
    duck.sendCmd(0,0)
   
    stime = time.time()
    while(time.time()-stime < 1):
        duck.sendCmd(0,0)
    stime = time.time()
    while(time.time()-stime < float(t)):
        duck.sendCmd(0,0.65*omega);
    print ("X: ", duck.x - x, " Y: ", duck.y -y)
    duck.sendCmd(0,0)
    stime = time.time()
    while(time.time()-stime < 1):
        duck.sendCmd(0,0)
    stime = time.time()
    while(time.time()-stime < float(t)):
        duck.sendCmd(float(vel),0.01775*omega);
    print ("X: ", duck.x - x, " Y: ", duck.y -y)
    duck.sendCmd(0,0)
    stime = time.time()
    while(time.time()-stime < 1):
        duck.sendCmd(0,0)
    stime = time.time()
    while(time.time()-stime < float(t)):
        duck.sendCmd(0,0.65*omega);
    print ("X: ", duck.x - x, " Y: ", duck.y -y)
    duck.sendCmd(0,0)
    """