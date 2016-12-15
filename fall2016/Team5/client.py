#!/usr/bin/env python
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy
import string
import time
import cv2
import pygame
import os
import sys

#VARIABLE INITIALIZATION:

laneFollow = 0
slotCar = 1
driverAssist = 2

s = open(os.devnull, 'w')

#USE THESE VALUES:
# kP = 20
# kI = .05
# kD = 0
# kPphi = 21
# kIphi = .065
# kDphi = 0
# maxval = 4.5
# controlgain = 0.75
# wheelgain = .3
#==================================================================
def laneControl(mode):
    kP = 20
    kI = 9
    kD = 0
    P = 0
    I = 0
    D = 1

    iMax = 1
    iMaxPhi = 1

    kPphi = 18.5
    kIphi = 9
    kDphi = 0
    aP = 0
    aI = 0
    aD = 0

    eP = 0
    eC = 0

    tR = 10
    tRP = 10
    #left lane is .13
    #inside lane is -.09

    phiTarget = 0

    maxval = 4.2

    controlgain = 0.75
    wheelgain = .25
    anglegain = 1 - controlgain 
    # print joystick.get_name()
    throttleGain = .7
    trimGain = 6
    avgSize = 2
    avgPhiSize = 2
    avg = 0;
    avgPhi = 0;
    threshold = 100000
    thresholdPhi = 10000

    assistThresholdlatch = 2
    assistThresholdrelease = 3

    left = .17
    right = -.04
    dTarget = right

    rollingAverage = [0];
    rollingPhiAverage = [0];

    output = []

    pygame.init()
    joysticks = initJoysticks()

    laneLatch = 1
    correcting = 0

    while (1):
        lastOutput = output
        output = joystickButtons(joysticks[0])
        axis = joystickAxis(joysticks[0])
        if(output[0] and output[0] != lastOutput[0]):
            laneLatch = not laneLatch
        if(laneLatch):
            dTarget = right
        else:
            dTarget = left
        lane_pose = duck.lane_pose
        if(abs(lane_pose.d - rollingAverage[-1]) > threshold):
            rollingAverage.append((lane_pose.d + rollingAverage[-1])/2)
        else:
            rollingAverage.append(lane_pose.d)
        if(len(rollingAverage) > avgSize):
            rollingAverage.pop(0)
        avg = 0;
        for i in range (0,len(rollingAverage)):
            avg += rollingAverage[i]
        avg = avg/avgSize

        eC = dTarget - avg

        P = eC * kP
        # if(P > .75):
        #     P = .75
        # if(P < -.75):
        #     P = -.75
        I = I + abs(eC) * kI
        I = I / tR
        if(I > iMax):
            I = iMax
        if(I < -iMax):
            I = -iMax


        if(abs(lane_pose.phi - rollingPhiAverage[-1]) > thresholdPhi):
            rollingAverage.append((lane_pose.d + rollingAverage[-1])/2)
        else:
            rollingPhiAverage.append(lane_pose.phi)
        if(len(rollingPhiAverage) > avgPhiSize):
            rollingPhiAverage.pop(0)
        avgPhi = 0;
        for i in range (0,len(rollingPhiAverage)):
            avgPhi += rollingPhiAverage[i]
        avgPhi = avgPhi/avgPhiSize



        eCP = 1*(phiTarget - avgPhi)
        aP = eCP * kPphi
        # if(P > .75):
        #     P = .75
        # if(P < -.75):
        #     P = -.75
        aI = aI + abs(eCP) * kIphi
        aI = aI / tRP
        if(aI > iMaxPhi):
            aI = iMaxPhi
        if(aI < -iMaxPhi):
            aI = -iMaxPhi

        cartesianCorrection = controlgain*(P+I)
        angularCorrection = anglegain*(aP+aI)
        correction = (cartesianCorrection + angularCorrection)
        preCap = correction
        if(correction > maxval):
            correction = maxval
        elif(correction < -maxval):
            correction = -maxval

        print preCap
        if(mode == driverAssist and correcting == 0):
            if(abs(preCap) > assistThresholdlatch):
                correcting = 1
                    
            else:
                speed = throttleGain * axis[5]
                correction = trimGain * (-1*(axis[0] - 0.5))
        else:
            if(mode == slotCar):
                speed = throttleGain * (axis[5])
                # correction = correction * speed
                if(speed == 0):
                    correction = 0
            else:
                speed = wheelgain

        print axis
        if(correcting == 1):
                speed = wheelgain
                if(waitForJoyZero(axis) and abs(preCap) < assistThresholdrelease):
                    correcting = 0

        duck.sendCmd(speed, correction)
#=======================================================================
def waitForJoyZero(axis):
    threshold = .2
    zeroValues = [0.5, 0.5, 0, 0.5, 0.5, 0]
    zero = 1
    for i in range(0,len(axis)):
        if(abs(axis[i] - zeroValues[i]) > threshold):
            print(axis[i])
            zero = 0
    if(zero):
        print "READY FOR DRIVER CONTROL"
    return zero


def initJoysticks():
    pygame.joystick.init()
    joysticks = [pygame.joystick.Joystick(x) for x in range (pygame.joystick.get_count())]
    for x in range (pygame.joystick.get_count()):
        joysticks[x].init()
    return joysticks



def joystickButtons(joystick):
        pygame.event.pump()
        buttons = joystick.get_numbuttons()
        out = [None] * buttons
        # print joystick.get_name()
        for i in range(0, buttons):
            out[i] = joystick.get_button(i)
        return out

def joystickAxis(joystick):
    pygame.event.pump()
    axis = joystick.get_numaxes()
    out = [None] * axis
    # print joystick.get_name()
    for i in range(0, axis):
        out[i] = (joystick.get_axis(i) + 1)/2
    return out

def checkLaneConditions(avg, avgPhi, dTarget):
    angleThreshold = 1
    horizThreshold = .56
    if(abs(avg - dTarget) > horizThreshold):
        return 0
    elif(abs(avgPhi) > angleThreshold):
        return 0
    else:
        print "READY FOR DRIVER CONTROL"
        return 1

if __name__ == '__main__':
    RRN.UseNumPy=True
    # Register a Local transport
    t1 = RR.LocalTransport()
    RRN.RegisterTransport(t1)

    # Register a TCP transpor
    t2 = RR.TcpTransport()
    RRN.RegisterTransport(t2)
    print "Connecting..."
    duck = RRN.ConnectService("tcp://raytech.local:8888/DuckiebotServer.raytech/Duckiebot")
    print "Connection Successful!"
    

    #======================================================================
    #   PROGRAM SELECTION:
    # Use the following syntax:
    # 
    # laneControl(mode)
    # 
    # mode specifies the drive mode:
    # 
    # laneFollow = Autonomous lane following + lane change (Use "A" button)
    # slotCar = Slot car mode (Lane Follow + Lane Change + Throttle Control)
    # driverAssist = Assisted Driving (Autonomous Takeover)
    #======================================================================

try:
    laneControl(driverAssist)

   
except KeyboardInterrupt:
    duck.sendCmd(0, 0)



