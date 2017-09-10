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

def f2b(i):
    return struct.pack('<f', i)


class DobotSerialInterface:

    serial_connection = None
    read_buffer = deque()
    current_status = None

    MOVE_MODE_JUMP = 0
    MOVE_MODE_JOINTS = 1  # joints move independent
    MOVE_MODE_LINEAR = 2  # linear movement

    def __init__(self, port_name='COM4', baud_rate=9600):
        thread.start_new_thread(self.read_loop, ())
        self.connect(port_name, baud_rate)

    def __del__(self):
        print "Closing  "
        if self.serial_connection is not None and self.serial_connection.isOpen():
            print "Closing serial connection"
            self.serial_connection.close()


    def connect(self, port_name='COM4', baud_rate=9600):
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

    def _send_absolute_command(self, cartesian, p1, p2, p3, p4, grip, move_mode):
        # global cmd_str_10
        cmd_str_10 = [0]*10
        cmd_str_10[0] = 3 if cartesian else 6  # position or angles
        cmd_str_10[2] = p1
        cmd_str_10[3] = p2
        cmd_str_10[4] = p3
        cmd_str_10[5] = p4
        cmd_str_10[7] = move_mode
        cmd_str_10[8] = grip
        self._send_command(cmd_str_10)

    def send_absolute_position(self, x, y, z, rot, grip, move_mode=MOVE_MODE_JOINTS):
        print "sending position %f %f %f" % (x, y, z)
        self._send_absolute_command(True, x, y, z, rot, grip, move_mode)

    def setJointPositions(self, base, rear, front, rot, grip):
        self.send_absolute_angles(base, rear, front, rot, grip)

    def getJointPositions(self):
        return self.current_status.angles
    def getPositions(self):
        return self.current_status.position
    def getGripperAngle(self):
        return self.current_status.gripperAngle

    def send_absolute_angles(self, base, rear, front, rot, grip, move_mode=MOVE_MODE_JOINTS):
        # todo: assertions for ranges
        self._send_absolute_command(False, base, rear, front, rot, grip, move_mode)

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


    def read_loop(self):
        #print "Entering loop"
        cnt = 0

        while True:

            # time.sleep(0.001)

            if self.serial_connection is None:
                print "Waiting for serial connection"
                time.sleep(0.5)
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
                if cnt % 10 == 0:
                    print datetime.datetime.now(), msg.angles
                cnt += 1
                # print cnt


# def dobot_cmd_send_9():
#     # global cmd_str_10

#
#
# def dobot_cmd_send_10(VelRat=100, AccRat=100):
