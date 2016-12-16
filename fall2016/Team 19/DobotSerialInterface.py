#! /usr/bin/env python
import thread
import struct
from collections import deque
import serial
import time
from DobotStatusMessage import DobotStatusMessage
import binascii
import datetime
import sys
import math
import RobotRaconteur as RR
import kinematics as kin


RRN = RR.RobotRaconteurNode.s




import time
import os.path

## Waits for a file to exist
def waitForFile(fname):
    while True:
        if os.path.isfile(fname):
            break
        time.sleep(1)



## Returns array of arrays of center of mass X, Y, bounding rectangle
## left, top, right, bottom.  also deletes the file
def getInput(fname):
    waitForFile(fname)

    f = open(fname)
    output = []
    XY = []

    for line in f:
        temp = line.split(",")
        CMx = temp[0]
        CMy = temp[1]
        BRl = temp[3]
        BRt = temp[4]
        BRr = temp[5]
        BRb = temp[6]
        output.append([CMx, CMy, BRl, BRt, BRr, BRb])
        XY.append([CMx, CMy])



    f.close()
    os.remove(fname)
    os.remove("Picture 1.jpg")
    return output, XY



def mapToPose(x,y):
    x = float(x)
    y = float(y)

    y = 480 - y

    tx = x - 320
    ty = y - 240

    ## Offset to center + pixel calculated + camera offset
    mmx = 0 + tx * (75.0 / 346.0) + 14
    mmy = 220 + ty * (75.0 / 346.0) + 45

    return [mmx, mmy, 12]

## 275px = 3cm
## 48mm from camera to suction cup
## 14mm in positive x













def f2b(i):
    return struct.pack('<f', i)


class DobotSerialInterface(object):
    
    serial_connection = None
    read_buffer = deque()
    current_status = None

    MOVE_MODE_JUMP = 0
    MOVE_MODE_JOINTS = 1  # joints move independent
    MOVE_MODE_LINEAR = 2  # linear movement
    
    base_rotation = 0

    def __init__(self, port_name='/dev/ttyACM0', baud_rate=9600):
        thread.start_new_thread(self.read_loop, ())
        thread.start_new_thread(self.run_commands, ())
        self.connect(port_name, baud_rate)

    def __del__(self):
        print "Closing  "
        if self.serial_connection is not None and self.serial_connection.isOpen():
            print "Closing serial connection"
            self.serial_connection.close()


    def connect(self, port_name='/dev/ttyACM0', baud_rate=9600):
        self.serial_connection = None
        try:
            self.serial_connection = serial.Serial(
                port=port_name,
                baudrate=baud_rate,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                bytesize=serial.EIGHTBITS
                ,timeout=0
            )
        except serial.SerialException as e:
            print "Could not connect", e

        time.sleep(2)  # no idea why but robot does not move if speed commands are sent
        # directly after opening serial

        while self.current_status is None:
            print "Waiting for status message"
            time.sleep(1)
        print "received first status message with position", self.current_status.position
        
        # Initialization functions
        try:
            print('Set speed')
            self.set_speed(50,50)
        except:
            print "Couldn't set speed"        

    def is_connected(self):
        return (self.serial_connection is not None) and self.serial_connection.isOpen()

    def _send_command(self, cmd_str_10):
        assert len(cmd_str_10) == 10

        if not self.is_connected():
            print "No serial connection"

        cmd_str_42 = ['\x00']*DobotStatusMessage.MESSAGE_LENGTH
        cmd_str_42[0] = '\xA5'
        cmd_str_42[-1] = '\x5A'
        for i in range(10):
            str4 = struct.pack('<f', float(cmd_str_10[i]))
            cmd_str_42[4 * i + 1] = str4[0]
            cmd_str_42[4 * i + 2] = str4[1]
            cmd_str_42[4 * i + 3] = str4[2]
            cmd_str_42[4 * i + 4] = str4[3]
        cmd_str = ''.join(cmd_str_42)
        self.serial_connection.write(cmd_str)
        # print "sending", binascii.b2a_hex(cmd_str)

    def _send_absolute_command(self, cartesian, p1, p2, p3, p4, p5):
        # global cmd_str_10
        cmd_str_10 = [0]*10
        cmd_str_10[0] = 3 if cartesian else 6  # position or angles
        cmd_str_10[2] = p1
        cmd_str_10[3] = p2
        cmd_str_10[4] = p3
        cmd_str_10[5] = p4
        cmd_str_10[6] = p5
        cmd_str_10[7] = self.MOVE_MODE_JOINTS
        self._send_command(cmd_str_10)
    
    def setAbsolutePosition(self, cartesian, p1, p2, p3, p4, p5):
        self._send_absolute_command(cartesian, p1, p2, p3, p4, p5)
        
    def setPos(self, x,y,z, rot, p5):
        
        if rot < -90:
            rot += 180
            
        elif rot > 90:
            rot -= 180
        
        t = kin.invkin(x,y,z)
        self.base_rotation = t[0]
        self.setJointPositions(t[0],t[1],t[2],rot,p5)
        
        TOLERANCE = 7
        
        counter = 0
        while True:
            temp = self.getJointPositions()
            counter += 1
            if abs(temp[0] - t[0]) < TOLERANCE:
                if abs(temp[1] - t[1]) < TOLERANCE:
                    if abs(temp[2] - t[2]) < TOLERANCE:
                        break
            time.sleep(0.05)
            if counter > 150:
                break
        

    def send_absolute_position(self, cartesian, x, y, z, rot, p5):
        print "sending position %f %f %f" % (x, y, z)
        self._send_absolute_command(True, x, y, z, rot, p5)

    def setJointPositions(self, base, rear, front, rot, p5):
        
        self.send_absolute_angles(base, rear, front, rot, p5)

    def send_absolute_angles(self, base, rear, front, rot, p5):
        # todo: assertions for ranges
        self._send_absolute_command(False, base, rear, front, rot, p5)

    def set_initial_angles(self, rear_arm_angle, front_arm_angle):
        print 'setting angles to', rear_arm_angle, front_arm_angle
        cmd_str_10 = [0]*10
        cmd_str_10[0] = 9
        cmd_str_10[1] = 3  # set initial angle
        cmd_str_10[2] = rear_arm_angle
        cmd_str_10[3] = front_arm_angle
        self._send_command(cmd_str_10)

    def apply_arm_angle_offsets(self, rear_arm_angle_offset, front_arm_angle_offset):

        while self.current_status is None:
            print "waiting for angle readings"
            # return False
            time.sleep(0.1)

        new_rear_angle = rear_arm_angle_offset + self.current_status.get_rear_arm_angle()
        new_front_angle = front_arm_angle_offset + self.current_status.get_front_arm_angle()

        print "front was", self.current_status.get_front_arm_angle(), "will be", new_front_angle
        print "back was", self.current_status.get_rear_arm_angle(), "will be", new_rear_angle

        # 90-x: not documented, but works
        self.set_initial_angles(90-new_rear_angle, new_front_angle)
        print "applied arm_angle offsets"
        return True

    def set_speed(self, VelRat=100, AccRat=100):
        cmd_str_10 = [0]*10
        cmd_str_10[0] = 10
        cmd_str_10[2] = VelRat
        cmd_str_10[3] = AccRat
        self._send_command(cmd_str_10)

    def set_playback_config(self):
        cmd_str_10 = [0]*10
        cmd_str_10[0] = 9
        cmd_str_10[1] = 1
        cmd_str_10[2] = 200  # JointVel
        cmd_str_10[3] = 200  # JointAcc
        cmd_str_10[4] = 200  # ServoVel
        cmd_str_10[5] = 200  # ServoAcc
        cmd_str_10[6] = 800  # LinearVel
        cmd_str_10[7] = 1000  # LinearAcc
        self._send_command(cmd_str_10)

    def getJointPositions(self):
        return self.current_status.angles





    def moveblock(self, init_coords, final_coords):
        x1 = init_coords[0]
        y1 = init_coords[1]
        z1 = init_coords[2]
        r1 = init_coords[3]
        
        x2 = final_coords[0]
        y2 = final_coords[1]
        z2 = final_coords[2]
        r2 = final_coords[3]
        
        self.setPos(0,150,40,r1-self.base_rotation,0)
        
        # Pick up the block
        self.setPos(x1,y1,z1+75,r1-self.base_rotation,0)
        self.setPos(x1,y1,z1+5,r1-self.base_rotation,0)
        self.setPos(x1,y1,z1,r1-self.base_rotation,1)
        self.setPos(x1,y1,z1+75,r1-self.base_rotation,1)
        self.setPos(x1,y1,z1+75,r2-self.base_rotation,1)
        
        # Move to over where it needs to go and drop
        self.setPos(x2,y2,z2+75,r2-self.base_rotation,1)
        self.setPos(x2,y2,z2+25,r2-self.base_rotation,1)
        self.setPos(x2,y2,z2+5,r2-self.base_rotation,1)
        self.setPos(x2,y2,z2,r2-self.base_rotation,0)
        self.setPos(x2,y2,z2+75,r2-self.base_rotation,0)
        self.setPos(0,150,50,0,0)
        
        
        
        



    def run_commands(self):
        cnt = 0
        
        while not self.is_connected():
            time.sleep(0.25)
            
        time.sleep(4)
        
        
        while True:
            #SUCTION CUP TEST
        
            if cnt == 30:
                self.setPos(0,220,160,0,0)


            if cnt == 60:
                x_l = 0
                y_l = 220

                for i in range(20):
                    print "IMAGE"
                    execfile("getImage.py")
                    coords = getInput("output.csv")[1]

                    x,y,z = mapToPose(coords[0][0], coords[0][1])
                    z = 125

                    # x_l += x
                    # y_l += y - 220

                    print "[" + str(x), str(y), str(z) + "]"
                    self.setPos(x, y, z,0,0)



                    time.sleep(3)







            ## Double stack layers with vision
            if cnt == -2:   #send_absolute_position(100,100,100,0,1)
                execfile("getImage.py")
                
                init = []
                fin = []

                x_out = -140
                y_out = 150
                z_out = 12



                for i in range(6):
                    temp = kin.getNthPosition(x_out,y_out,z_out,i)
                    fin.append([temp[0],temp[1],temp[2],temp[3]])


                coords = getInput("output.csv")[1]
                for c in coords:
                    x,y,z = mapToPose(c[0], c[1])
                    init.append([x,y,z,0])

                done = 0;
                for i in range(len(init)):
                    done += 1
                    # fin[i][3] -= (i+1)*2
                    if i > 2:
                        fin[i][0] += 5
                        fin[i][1] += 15 + 4*int(i)/4 
                    self.moveblock(init[i], fin[i])

                self.setPos(0,220,160,0,0)

                # time.sleep(3)

                init = []

                execfile("getImage.py")
                coords = getInput("output.csv")[1]
                for c in coords:
                    x,y,z = mapToPose(c[0], c[1])
                    init.append([x,y,z,0])

                for i in range(len(init)):
                    self.moveblock(init[i], fin[i+done])

                self.setPos(0,220,160,0,0)


            # if cnt == 90:
            #     self.setPos(0,220,160,0,0)


            # if cnt==120:   #send_absolute_position(100,100,100,0,1)
            #     execfile("getImage.py")
            #     coords = getInput("output.csv")[1][0]
            #     x,y,z = mapToPose(coords[0], coords[1])
            #     print "X,Y,Z = ",str(x),str(y),str(z)
            #     self.setPos(x,y,z,0,0)

            # if cnt == 150:
            #     self.setPos(0,220,160,0,0)

            # if cnt==180:   #send_absolute_position(100,100,100,0,1)
            #     execfile("getImage.py")
            #     coords = getInput("output.csv")[1][0]
            #     x,y,z = mapToPose(coords[0], coords[1])
            #     print "X,Y,Z = ",str(x),str(y),str(z)
            #     self.setPos(x,y,z,0,0)

            # if cnt == 210:
            #     self.setPos(0,220,160,0,0)
            
            #if cnt==100:   #send_absolute_position(100,100,100,0,1)
                    #self.setPos(-4.5,272,15,0,0)
            
           
        
            cnt += 1            






    def read_loop(self):
        #print "Entering loop"

        cnt = 0
        
        
        while True:

            # time.sleep(0.001)
            
            #self.setPos(x,y,z,rot,p5)
            
            if self.serial_connection is None:
                print "Waiting for serial connection"
                time.sleep(0.5)
                cnt = 0
                continue

            r = self.serial_connection.read(200)  # TODO: select better count and read-type (see init)
            # print "read", len(r), "chars"

            ascii = binascii.b2a_hex(r)
            for i in range(len(ascii) / 2):
                self.read_buffer.append(ascii[2 * i] + ascii[2 * i + 1])

            n = len(self.read_buffer)
            if n < DobotStatusMessage.MESSAGE_LENGTH:
                continue

            # should not be triggered
            # if len(self.read_buffer) > 100:
            #     print "read Buffer is expanding too fast"

            # remove stuff in front of 'A5'
            while len(self.read_buffer):
                s = self.read_buffer[0]
                if s == 'a5':
                    break
                self.read_buffer.popleft()

            n_cleaned = len(self.read_buffer)

            # if n-n_cleaned > 0:
            #     print "Removed", (n-n_cleaned), "characters"

            while len(self.read_buffer) >= DobotStatusMessage.MESSAGE_LENGTH:
                message = list()
                # print "buffer size", len(self.read_buffer)
                for i in range(DobotStatusMessage.MESSAGE_LENGTH):
                    message.append(self.read_buffer.popleft())

                # sanity check
                if message[-1] != '5a':
                    # print "Message was not terminated by '5a', but", message[-1], "ignoring message"
                    continue

                msg = DobotStatusMessage()
                msg.parse_ascii(message)

                # mutex?
                self.current_status = msg

                # sys.stdout.flush()
                
            
                # print cnt


def main():    

    port = 10001       
    t1 = RR.LocalTransport()
    t1.StartServerAsNodeName("dobotRR")
    RRN.RegisterTransport(t1)

    t2 = RR.TcpTransport()
    t2.EnableNodeAnnounce()
    t2.StartServer(port)
    RRN.RegisterTransport(t2)
    
    my_dobot = DobotSerialInterface('COM5')

    with open('dobotRR.robodef', 'r') as f:
        service_def = f.read()
    
    RRN.RegisterServiceType(service_def)
    RRN.RegisterService("dobotController", "dobotRR.DobotSerialInterface", my_dobot)
    print "Conect string: tcp://localhost:" + str(port) + "/dobotRR/dobotController"
    raw_input("Press any key to end")

    RRN.Shutdown()


if __name__ == '__main__':
    main()

