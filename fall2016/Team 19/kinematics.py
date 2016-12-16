import numpy as np
import math

HORIZONTAL_ENDEFFECTOR = 59.0
ARM2 = 160

## Forward Kinematics
def triangle(q1, q2):
    l1 = 135.0
    l2 = ARM2
    q1 = math.radians(90-q1)
    q2 = math.radians(q2)

    y = l1 * math.cos(q1) + l2 * math.cos(q2)
    z = l1 * math.sin(q1) - l2 * math.sin(q2)

    return [0,y+HORIZONTAL_ENDEFFECTOR,z+100-68]

def rotz(v, theta):
    theta = math.radians(theta)
    z = np.array([0,0,1])
    v = np.array(v)
    rot = v * math.cos(theta) + np.cross(z,v) * math.sin(theta) + z * np.dot(z,v)*(1-math.cos(theta))
    return rot


def fwdkin(q1, q2, q3):
    return rotz(triangle(float(q1),float(q2)),float(q3))



## Inverse Kinematics

def invkin(x,y,z):
    l1 = 135.0
    l2 = ARM2
    z -= 100.0 - 68

    x = float(x)
    y = float(y)
    z = float(z)

    q1 = math.atan(-x/y)

    y -= HORIZONTAL_ENDEFFECTOR

    l = math.sqrt(x*x + y*y)

    D = math.sqrt(l*l + z*z)

    theta1 = math.acos((l2*l2 - l1*l1 - D*D) / (-2 * l1 * D))
    theta2 = math.acos((D*D - l1*l1 - l2*l2) / (-2 * l1 * l2))
    theta3 = math.acos((l1*l1 - D*D - l2*l2) / (-2 * D * l2))

    # print D
    # print math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)

    q2 = math.pi / 2 - (theta1 + math.atan(z / l))
    q3 = -1 * (theta2 - q2 - math.pi/2)


    return [math.degrees(q1), math.degrees(q2), math.degrees(q3)]




## Function takes in the position of the first block
## in the jenga tower and the number of the block that
## is currently being placed and returns the position
## of that block and if it is rotated or not relative
## to the first block.  Assumes that the AR tags are
## placed in the center of the jenga pieces
def getNthPosition(x_in, y_in, z_in, n):
    ## Dimensions of jenga block
    ## Need to be changed
    LENGTH = 74.8
    WIDTH = 24.2
    HEIGHT = 14.1

    ## Output coordinates
    x = 0
    y = 0
    z = 0
    rotation = 0

    ## Height increases by one unit of height
    ## every third block
    z = HEIGHT * (n/3)

    ## Rotation changes every third block
    rotation = (n/3) % 2

    ## First 3 blocks only x changes
    if n % 6 < 3:
        x = WIDTH * (n%3)

    ## Second set of 3 blocks. x is in the middle
    ## of the tower. ((n%3)-1) = -1,0,1 depending
    ## on n, so it will a third of the length down,
    ## up, or not at all
    else:
        x = WIDTH
        y = ((n%3)-1) * (LENGTH/3.0)

    ## Return values plus the initital coordinate
    return [x+x_in, y+y_in, z+z_in, 90*rotation]