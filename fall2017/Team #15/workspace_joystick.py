#!/usr/bin/env python

"""
Baxter joystick position control workspace
Author: Garrison Johnston
"""
import argparse

import rospy

from baxter_pykdl import baxter_kinematics

import baxter_interface
import baxter_external_devices
from baxter_interface import CHECK_VERSION

from operator import add

import math

def reset_arm(cmd, joints):
    """
    Resets arm to zero configuration
    """
    reset_pose = [0, -math.pi/4, 0, math.pi/4, 0, math.pi/2, 0]
    j=0
    for i in joints:
        joint = joints[j]
        cmd[joint] = reset_pose[j]
        j+=1
    


def set_j(cmd, direction, joints):
    """
    Set the selected joint to current pos + delta.

    @param cmd: the joint command dictionary
    @param direction: unit vector (x, y, or z)
    @param joints: list of joints

    joint/index is to make this work in the bindings.
    """
    neutral_quaternion = [0.49848584, 0.86663104, 0.02108932, -0.00421412]
    kin = baxter_kinematics('right')
    fk = kin.forward_position_kinematics()
    fk_new = map(add, fk, direction) 
    ik = kin.inverse_kinematics(fk_new[0:3], neutral_quaternion)  # position & orientation
    if ik is None:
        print "Movement not possible"
    else:
        j=0
        for i in joints:
            joint = joints[j]
            cmd[joint] = ik[j]
            j+=1

def map_joystick(joystick):
    """
    Maps joystick input to joint position commands.

    @param joystick: an instance of a Joystick
    """
    left = baxter_interface.Limb('left')
    right = baxter_interface.Limb('right')
    grip_left = baxter_interface.Gripper('left', CHECK_VERSION)
    grip_right = baxter_interface.Gripper('right', CHECK_VERSION)
    lcmd = {}
    rcmd = {}

    # Directions
    x = [0.05, 0, 0, 0, 0, 0, 0]
    x_neg = [-0.05, 0, 0, 0, 0, 0, 0]
    y = [0, 0.05, 0, 0, 0, 0, 0]
    y_neg = [0, -0.05, 0, 0, 0, 0, 0]
    z = [0, 0, 0.05, 0, 0, 0, 0]
    z_neg = [0, 0, -0.05, 0, 0, 0, 0]
    
    #available joints
    lj = left.joint_names()
    rj = right.joint_names()

    #abbreviations
    jhi = lambda s: joystick.stick_value(s) > 0
    jlo = lambda s: joystick.stick_value(s) < 0
    bdn = joystick.button_down
    bup = joystick.button_up

    def print_help(bindings_list):
        print("Press Ctrl-C to quit.")
        for bindings in bindings_list:
            for (test, _cmd, doc) in bindings:
                if callable(doc):
                    doc = doc()
                print("%s: %s" % (str(test[1][0]), doc))

    bindings_list = []
    bindings = (
        ((bdn, ['rightTrigger']),
            (grip_left.close,  []), "left gripper close"),
        ((bup, ['rightTrigger']),
            (grip_left.open,   []), "left gripper open"),
        ((bdn, ['leftTrigger']),
            (grip_right.close, []), "right gripper close"),
        ((bup, ['leftTrigger']),
            (grip_right.open,  []), "right gripper open"),
        ((jlo, ['leftStickHorz']),
            (set_j, [rcmd, y, rj]), "+y"),
        ((jhi, ['leftStickHorz']),
            (set_j, [rcmd, y_neg, rj]), "-y"),
        ((jlo, ['leftStickVert']),
            (set_j, [rcmd, x, rj]), "+x"),
        ((jhi, ['leftStickVert']),
            (set_j, [rcmd, x_neg, rj]), "-x"),
        ((jlo, ['rightStickVert']),
            (set_j, [rcmd, z_neg, rj]), "-z"),
        ((jhi, ['rightStickVert']),
            (set_j, [rcmd, z, rj]), "+z"),
        ((bdn, ['rightBumper']),
            (reset_arm, [rcmd, rj]), "Reset arm"),
        ((bdn, ['leftBumper']),
            (reset_arm, [rcmd, rj]), "Reset Arm"),
        ((bdn, ['btnRight']),
            (grip_left.calibrate, []), "left calibrate"),
        ((bdn, ['btnLeft']),
            (grip_right.calibrate, []), "right calibrate"),
        ((bdn, ['function1']),
            (print_help, [bindings_list]), "help"),
        ((bdn, ['function2']),
            (print_help, [bindings_list]), "help"),
        )
    bindings_list.append(bindings)

    rate = rospy.Rate(100)
    # print_help(bindings_list)
    
    print("Press Ctrl-C to stop. ")

    while not rospy.is_shutdown():
        for (test, cmd, doc) in bindings:
            if test[0](*test[1]):
                cmd[0](*cmd[1])
                if callable(doc):
                    print(doc())
                else:
                    print(doc)
        if len(lcmd):
            left.set_joint_positions(lcmd)
            lcmd.clear()
        if len(rcmd):
            right.set_joint_positions(rcmd)
            rcmd.clear()
        rate.sleep()
    return False

def main():
    epilog = """
See help inside the example with the "Start" button for controller
key bindings.
    """
    arg_fmt = argparse.RawDescriptionHelpFormatter
    parser = argparse.ArgumentParser(formatter_class=arg_fmt,
                                        description=main.__doc__,
                                        epilog=epilog)
    required = parser.add_argument_group('required arguments')
    required.add_argument(
        '-j', '--joystick', required=True,
        choices=['xbox', 'logitech', 'ps3'],
        help='specify the type of joystick to use'
    )
    args = parser.parse_args(rospy.myargv()[1:])

    joystick = None
    if args.joystick == 'xbox':
        joystick = baxter_external_devices.joystick.XboxController()
    elif args.joystick == 'logitech':
        joystick = baxter_external_devices.joystick.LogitechController()
    elif args.joystick == 'ps3':
        joystick = baxter_external_devices.joystick.PS3Controller()
    else:
        parser.error("Unsupported joystick type '%s'" % (args.joystick))

    print("Initializing node... ")
    rospy.init_node("rsdk_joint_position_joystick")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled

    def clean_shutdown():
        print("\nExiting example.")
        if not init_state:
            print("Disabling robot...")
            rs.disable()
    rospy.on_shutdown(clean_shutdown)

    print("Enabling robot... ")
    rs.enable()

    map_joystick(joystick)
    print("Done.")

if __name__ == '__main__':
    main()
