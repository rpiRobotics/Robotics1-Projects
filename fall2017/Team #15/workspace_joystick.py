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


def rotate(l):
    """
    Rotates a list left.

    @param l: the list
    """
    if len(l):
        v = l[0]
        l[:-1] = l[1:]
        l[-1] = v


def set_j(cmd, direction, joints):
    """
    Set the selected joint to current pos + delta.

    @param cmd: the joint command dictionary
    @param direction: unit vector (x, y, or z)
    @param joints: list of joints

    joint/index is to make this work in the bindings.
    """
    kin = baxter_kinematics('right')
    fk = kin.forward_position_kinematics()
    fk_new = map(add, fk, direction) 
    ik = kin.inverse_kinematics(fk_new[0:3], fk_new[3:8])  # position & orientation
    print ik
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
    x = [0.1, 0, 0, 0, 0, 0, 0]
    x_neg = [-0.1, 0, 0, 0, 0, 0, 0]
    y = [0, 0.1, 0, 0, 0, 0, 0]
    y_neg = [0, -0.1, 0, 0, 0, 0, 0]
    z = [0, 0, 0.1, 0, 0, 0, 0]
    z_neg = [0, 0, -0.1, 0, 0, 0, 0]
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
         (set_j, [rcmd, x, rj]), lambda: "right inc " + rj[0]),
        ((jhi, ['leftStickHorz']),
         (set_j, [rcmd, x_neg, rj]), lambda: "right dec " + rj[0]),
        # ((jlo, ['rightStickHorz']),
        #  (set_j, [lcmd, left,  lj, 0,  0.1]), lambda: "left inc " + lj[0]),
        # ((jhi, ['rightStickHorz']),
        #  (set_j, [lcmd, left,  lj, 0, -0.1]), lambda: "left dec " + lj[0]),
        ((jlo, ['leftStickVert']),
         (set_j, [rcmd, y, rj]), lambda: "right inc " + rj[1]),
        ((jhi, ['leftStickVert']),
         (set_j, [rcmd, y, rj]), lambda: "right dec " + rj[1]),
        ((jlo, ['rightStickVert']),
         (set_j, [rcmd, z, rj]), lambda: "left inc " + lj[1]),
        ((jhi, ['rightStickVert']),
         (set_j, [rcmd, z_neg, rj]), lambda: "left dec " + lj[1]),
        ((bdn, ['rightBumper']),
         (rotate, [lj]), "left: cycle joint"),
        ((bdn, ['leftBumper']),
         (rotate, [rj]), "right: cycle joint"),
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
    """RSDK Joint Position Example: Joystick Control

    Use a game controller to control the angular joint positions
    of Baxter's arms.

    Attach a game controller to your dev machine and run this
    example along with the ROS joy_node to control the position
    of each joint in Baxter's arms using the joysticks. Be sure to
    provide your *joystick* type to setup appropriate key mappings.

    Each stick axis maps to a joint angle; which joints are currently
    controlled can be incremented by using the mapped increment buttons.
    Ex:
      (x,y -> e0,e1) >>increment>> (x,y -> e1,e2)
    """
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
