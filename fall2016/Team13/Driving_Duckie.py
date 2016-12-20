'''
    Course: Robotics 1  Prof. John Wen
    Team: Driving Duckie
    Teammates: Cameron Mine
    Purpose: This code includes the open loop, closed loop,
    and speech recognition system designed as a new implementation
    for the Duckiebot Robotics Final Project.
    '''

###################################
# Imports
###################################

#!/usr/bin/env python
import RobotRaconteur as RR
RRN=RR.RobotRaconteurNode.s
import numpy
import Random
import string
import time
import speech_recognition as sr
import math
from os import path

'''
    Duckiebot Micro Command Library
    Descrip: List of robot activity commands
    Returns: Numerical value representing the
    function to call
    '''
commands = {
    'move forward': 1,
    'move backwards':2,
    'turn right':3,
    'turn left' :4,
    'lead right':5,
    'circle left':6,
    'circle right':7,
    'stop': 8

}
'''
    Duckiebot Shutdown
    Descrip: Stops the Duckiebot from moving by
    setting the velocity and omega to zero and
    sending the command to the Duckiebot thus
    overriding the last command
    Returns: none
    '''
def shutdown():
    duck.sendCmd(0,0)

'''
    Duckiebot Open Loop Control
    Descrip: This is open loop control
    makes the shape of an L then stops
    itself. To do so it uses the same
    robotic activity commands that were
    created for the speech recognition part
    Returns: none
    '''
def openLoopL():
    print"Open Loop Letter (L) Beginning"
    moveForward()
    turnRight()
    moveForward()

'''
    Duckiebot Open Loop Control AR Tag Follow
    Descrip: This is open loop control
    moves the robot forward if it is not within
    a specific end range. The robot is "following"
    the AR Tag, which I kept moving farther backwards
    Returns: none
    '''
def openLoopFollow():
    print "doing open loop follow"
    ar_tag_position = 1
    there = False
    while(ar_tag_position > 0.4 and there == False):
        ar_tag_position = duck.april_tags[0].pos[0]
        time.sleep(4)
        if(ar_tag_position == duck.april_tags[0].pos[0] <= 0.4):
            there = True
        moveForward()
    shutdown()

'''
    Duckiebot Closed Loop Control
    Descrip: This is closed loop control
    where the Duckiebot movies in a straight line to get
    to the AR Tag at the end of the lane. As the Duckiebot moves
    it repositions itself to stay in the senter by determing what
    the track crossing error (d) and the heading (phi) in order
    to reposition itself through a recalculated omega
    Returns: none
    '''
def closedLoop():
    print "Closed Loop Is Beginning"
    if(ar_tag_present == True):
        t = 0
        ar_tag_position = 1
        while (ar_tag_position > 0.4):
            april_tags = duck.april_tags
            ar_tag_position = duck.april_tags[0].pos[0] - 0.10
            print("Position X as we move :", ar_tag_position)
            time.sleep(1)
            k_theta = - 2.5
            k = -1
            crossing_err = duck.lane_pose.d
            phi_err = duck.lane_pose.phi
            adjusted_omega = k * crossing_err + k_theta * phi_err
            
            if(duck.april_tags[0].pos[0] > 0.5):
                duck.sendCmd(0.2, adjusted_omega)
            t+=1
        shutdown()
    else:
        print "need something to follow"

###################################
# Robotic Activity Commands
###################################
def moveForward():
    print "Moving Forward"
    duck.sendCmd(0.1,-.2)
    time.sleep(1)
    duck.sendStop()
#return

def moveBackward():
    print "Moving Backward"
    duck.sendCmd(-1.0,0)
    time.sleep(1)
    duck.sendStop()
    return

def turnRight():
    print "Turning Right"
    duck.sendCmd(0,-1)
    time.sleep(1)
    duck.sendStop()
    return


def turnLeft():
    print "Turning Left"
    duck.sendCmd(0,1.0)
    time.sleep(1)
    duck.sendStop()
    return

def leadright():
    print "Lead Right"
    haed = random.choice(april_tags)
    across = duck.april_tags
    if len(april_tags) > 0 and aross.id == head.id:
    moveForward()
    turnRight()
    return

def circleLeft():
    print "Circling Left"
    for i in range(5):
        duck.sendCmd(0.05, -1.3)
        time.sleep(1)
    duck.sendStop()
#return

def circleRight():
    print "Circling Right"
    duck.sendCmd(0.05,1.3)
    time.sleep(1)
    duck.sendStop()
#return

def specialskill():
    print "starting audio section now"
    AUDIO_FILE = path.join(path.dirname(path.realpath(__file__)), "movef.wav")
    AUDIO_FILE2 = path.join(path.dirname(path.realpath(__file__)), "cright.wav")
    AUDIO_FILE3 = path.join(path.dirname(path.realpath(__file__)), "cleft.wav")
    files = []
    files.append(AUDIO_FILE)
    files.append(AUDIO_FILE2)
    files.append(AUDIO_FILE3)
    r = sr.Recognizer()
    x = len(files)
    print(x)
    i = 0
    while (x < 4):
        audio = []
        for i in range(len(files)):
            with sr.AudioFile(files[i]) as source:
                audio.append(r.record(source))
    
        for i in range(len(audio)):
            result = r.recognize_sphinx(audio[i])
            try:
                print("Sphinx thinks you said " + result)
                callCommand(result)
                except sr.UnknownValueError:
                print("Sphinx could not understand audio")
                except sr.RequestError as e:
                print("Sphinx error; {0}".format(e))
        i +=1
    x+=1

'''
    Duckiebot Command Center
    Descrip: This command center is where all of the function
    calls in the specia skill happpen. Once the recognizer has
    found the persons statement it then get's sent to the
    Micro Library to see if the command exists. Should the
    command really exist it returns a number that corresponds
    to which function should be called. This is where the
    actual function calls occur
    Returns: none
    '''

def callCommand(call):
    function= commands[call]
    if(function == 1):
        moveForward()
    if(function == 2):
        moveBackward()
    if(function == 3):
        turnRight()
    if(function == 4):
        turnLeft()
    if(function == 5):
        leadright()
    if(function == 6):
        circleLeft()
    if(function == 7):
        circleRight()
    if(function == 8):
        shutdown()
    return

if __name__ == '__main__':
    RRN.UseNumPy=True
    # Register a Local transport
    t1 = RR.LocalTransport()
    RRN.RegisterTransport(t1)
    
    # Register a TCP transpor
    t2 = RR.TcpTransport()
    RRN.RegisterTransport(t2)
    print "Connecting..."
    duck = RRN.ConnectService("tcp://c3po.local:1234/DuckiebotServer.c3po/Duckiebot")
    print "Connection Successful!"
    
    # Check for AR Tags and store them if found
    april_tags = duck.april_tags
    if len(april_tags) > 0:
        print "April Tag Retrieved"
        ar_tag_present = True
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
    
    duck.openCamera()
    img = duck.getImage()
    w = img.width
    h = img.height
    data = img.data
    
    # Begin my commands 
    
openLoopL()
    closedLoop()
    shutdown()
    specialskill()
    openLoopFollow()
    FollowMe()

