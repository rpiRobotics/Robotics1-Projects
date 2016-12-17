# Kinematics module for the dobot
import numpy as np
import numpy.testing as npt

import math3D

# np.seterr(invalid='ignore')

# Arm parameters
l1 = 135 # length of arm link 1
l2 = 160 # length of arm link2 2
d = 55 # length of end effector

limits = np.array([[-135,135],[0,60],[0,60]])

# Vertices
#import itertools as it
#it.product([-20,10],[-5,5],[0,135])
link1 = np.transpose(np.matrix([[10.0,-20,0],[-50,-20,0],[10,20,0],[-50,20,0], \
    [10,-20,l1],[-50,-20,l1],[10,20,l1],[-50,20,l1]]))
link2 = np.transpose(np.matrix([[10.0,-15,0],[-50,-15,0],[10,15,0],[-50,15,0], \
    [10,-15,l2],[-50,-15,l2],[10,15,l2],[-50,15,l2]]))
hand1 = np.transpose(np.matrix([[0.0,-8,-67],[0,-8,15],[0,8,-67],[0,8,15], \
    [75,-8,-67],[75,-8,15],[75,8,-67],[75,8,15]]))
hand2 = np.transpose(np.matrix([[75.0,-40,-50],[75,-40,50],[75,30,-50],[75,30,50], \
    [105,-40,-50],[105,-40,50],[105,30,-50],[105,30,50]]))
hand = np.hstack((hand1,hand2))

# Faces defined relative to concatonated vertex list
tube = np.array([[0,1,4],[1,4,5],[1,3,5],[3,5,7],[3,2,7],[2,7,6],[2,0,6],[0,6,4]])
faces = np.vstack(([[0,1,3],[0,2,3]],tube,tube+8,tube+16,tube+24,[[20,21,23],[20,22,23]]))

# Precompute some quantities
l1_sq = pow(l1,2)
l2_sq = pow(l2,2)

def valid_angles(angles):
    """
    Returns True if the given angles are within the Dobot's range of motion
    """
    if any(np.isnan(angles)):
        return False
    # Rotation angle limits (measured limits are larger: -180 to 180)
    if (angles[0] < limits[0,0]) or (limits[0,1] < angles[0]):
        return False
    # Joint angle 1
    if (angles[1] < limits[1,0]) or (limits[1,1] < angles[1]):
        return False
    # Joint angle 2
    if (angles[2] < limits[2,0]) or (limits[2,1] < angles[2]):
        return False
    # Relative joint angle 2
    if ((angles[2] - angles[1]) < -35):
        return False
    return True

def forward_kinematics(angles):
    """
    Input: (a0,a1,a2)
        a0 - angle about base
        a1 - angle of first link from verticle
        a2 - angle of second link from horizontal
    Output:
        v - x,y,z coordinate of the end effector (3,)
    """
    r = l1*math3D.sind(angles[1]) + l2*math3D.cosd(angles[2]) + d
    v = np.zeros(3)
    v[0] = r*math3D.cosd(angles[0]) # x
    v[1] = r*math3D.sind(angles[0]) # y
    v[2] = l1*math3D.cosd(angles[1]) - l2*math3D.sind(angles[2]) # z
    return v

def R0T(angles):
    """
    Input: (a0,a1,a2)
    Output: rotation matrix to transform vectors in the end effector frame to the base frame
    """
    return math3D.rotz(angles[0])

def inverse_kinematics(v):
    """
    Input:
        v - x,y,z coordinate of the end effector (3,)
    Output: (a0,a1,a2)
        a0 - angle about base
        a1 - angle of first link from verticle
        a2 - angle of second link from horizontal
    """
    x = v[0]
    y = v[1]
    z = v[2]
    # pre-compute distances
    r = np.sqrt(pow(x,2) + pow(y,2)) # radial position of end effector in the x-y plane
    rho_sq = pow(r-d,2) + pow(z,2)
    rho = np.sqrt(rho_sq) # distance b/w the ends of the links joined at the elbow

    # law of cosines
    alpha = math3D.acosd((l1_sq + rho_sq - l2_sq)/(2.0*l1*rho))
    beta = math3D.atan2d(z,r-d)
    gamma = math3D.acosd((l1_sq + l2_sq - rho_sq)/(2.0*l1*l2))

    # joint angles
    a0 = math3D.atan2d(y,x)
    a1 = 90.0 - beta - alpha
    a2 = 90.0 - gamma + a1

    # return nan for the angles if the position is unreachable
    angles = (a0,a1,a2)
    if not valid_angles(angles):
        angles = (np.nan,np.nan,np.nan)
    return angles

def jacobian(angles):
    """
    Input: (a0,a1,a2)
    Output: Jacobian matrix d(x,y,z)/d(a0,a1,a2)
    """
    s0 = math3D.sind(angles[0])
    c0 = math3D.cosd(angles[0])
    s1 = math3D.sind(angles[1])
    c1 = math3D.cosd(angles[1])
    s2 = math3D.sind(angles[2])
    c2 = math3D.cosd(angles[2])
    r = l1*s1 + l2*c2 + d
    return np.matrix([ \
        [-r*s0, l1*c0*c1, -l2*c0*s2], \
        [ r*c0, l1*s0*c1, -l2*s0*s2], \
        [    0,   -l1*s1,    -l2*c2]])

def get_mesh(angles):
    """
    Transforms the arm model into a single triangle mesh in the global reference frame.
    """

    # Transform the model components into the global reference frame
    R0 = math3D.rotz(angles[0])
    R1 = math3D.roty(angles[1])
    R2 = math3D.roty(angles[2]+90.0)
    p1 = np.matrix([[0],[0],[l1]])
    p2 = np.matrix([[0],[0],[l2]])
    verts = np.transpose(np.array( \
        np.hstack(( R0*R1*link1 , R0*R2*link2+R0*R1*p1 , R0*(hand+(R2*p2+R1*p1)) )) \
        ))

    # Construct the array of triangles (arrays of vertices)
    arm = np.zeros([len(faces),3,3])
    for k in xrange(len(faces)):
        arm[k] = verts[faces[k]]
    return arm

def test():
    s2 = np.sqrt(2)
    check = npt.assert_array_almost_equal

    # test forward kinematics
    check(forward_kinematics((0,0,0)),[l2+d,0,l1])
    check(forward_kinematics((0,90,0)),[l1+l2+d,0,0])
    check(forward_kinematics((0,45,-45)), np.array([l1+l2,0,l1+l2])/np.sqrt(2) + [d,0,0])
    check(forward_kinematics((0,0,90)),[d,0,l1-l2])
    check(forward_kinematics((0,0,45)), [d+l2/np.sqrt(2),0,l1-l2/np.sqrt(2)])
    check(forward_kinematics((90,90,0)),[0,l1+l2+d,0])
    check(forward_kinematics((45,90,0)), np.array([l1+l2+d,l1+l2+d,0])/np.sqrt(2))

    # test inverse kinematics
    check(inverse_kinematics(forward_kinematics((0,0.01,0.01))),(0,0.01,0.01))
    check(inverse_kinematics(forward_kinematics((0,22.5,0.01))),(0,22.5,0.01))
    check(inverse_kinematics(forward_kinematics((0,45,45))),(0,45,45))
    check(inverse_kinematics(forward_kinematics((0,0.01,45))),(0,0.01,45))
    check(inverse_kinematics(forward_kinematics((0,0.01,22.5))),(0,0.01,22.5))
    check(inverse_kinematics(forward_kinematics((90,45,22.5))),(90,45,22.5))
    check(inverse_kinematics(forward_kinematics((45,45,22.5))),(45,45,22.5))

    # test jacobian
    check(jacobian((0,0,0)), np.matrix([[0,l1,0],[l2+d,0,0],[0,0,-l2]]))
    check(jacobian((90,90,0)), np.matrix([[-l1-l2-d,0,0],[0,0,0],[0,-l1,-l2]]))
    check(jacobian((0,45,45)), np.matrix([[0,l1/s2,-l2/s2],[(l1+l2)/s2+d,0,0],[0,-l1/s2,-l2/s2]]))
