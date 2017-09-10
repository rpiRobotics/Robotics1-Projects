# Class to facilitate keyboard control of the Dobot

import time
import numpy as np

import SerialInterface
import DobotModel

class Controller:
    max_speed = 5

    def __init__(self,port):
        """
        Input: port - the serial port connected to the Dobot
        """
        self.mode = 0 # absolute (0) or jog (1)
        self.effort = 30 # percent effort
        self.angles = [0.0,5.0,5.0]

        # Initialize serial interface
        self.interface = SerialInterface.SerialInterface(port)
        time.sleep(1)
        self.interface.send_absolute_angles(self.angles[0], self.angles[1], self.angles[2], 0.0)
        time.sleep(1)

    def switch_modes(self):
        """
        Switch between absolute (position) and jog (angle) modes.
        """
        if (self.mode == 1):
            self.interface.send_jog_command(False,0,0)
            time.sleep(1)
            self.angles = self.interface.current_status.angles[0:3]
        self.mode = (self.mode + 1) % 2

    def change_effort(self,de):
        """
        Input: de - change in effort (expressed as a percent of maximum effort)
        """
        self.effort = max(0,min(100,self.effort+de))

    def stop(self):
        """
        Signals the end of a jog command.
        """
        if (self.mode == 1):
            self.interface.send_jog_command(False,0,0)

    def get_angles(self):
        """
        Output: [a0,a1,a2]
        """
        return self.interface.current_status.angles[0:3]

    def get_position(self):
        """
        Output: [x,y,z]
        """
        return self.interface.current_status.position[0:3]

    def is_connected(self):
        """
        Output: True if the serial interface is still connected
        """
        return self.interface.is_connected()

    def move(self,direction):
        """
        Input: direction - code between 1 and 6 specifying the direction of the movement
            Dir Mode0 Mode1
            1   +y    +a0
            2   -y    -a0
            3   +x    +a1
            4   -x    -a1
            5   -z    +a2
            6   +z    -a2
        """
        if (self.mode == 1):
            self.interface.send_jog_command(False,direction,self.effort)
        else: # (mself.ode == 0):
            speed = self.max_speed*(self.effort/100.0)
            dp = np.zeros(3)
            if (direction == 1):
                dp[1] = +speed
            elif (direction == 2):
                dp[1] = -speed
            elif (direction == 3):
                dp[0] = +speed
            elif (direction == 4):
                dp[0] = -speed
            elif (direction == 5):
                dp[2] = -speed
            elif (direction == 6):
                dp[2] = +speed

            # Use kinematics to adjust the x,y,z position
            p = DobotModel.forward_kinematics(self.angles)
            p += dp
            a_new = DobotModel.inverse_kinematics(p)
            if not any(np.isnan(a_new)):
                self.angles = a_new
                self.interface.send_absolute_angles(self.angles[0], self.angles[1], self.angles[2], 0.0)
