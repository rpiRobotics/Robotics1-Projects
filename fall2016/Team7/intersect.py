# 3D collision detection module for triangles

import numpy as np
import numpy.testing as npt
import itertools as it
#import matplotlib.pyplot as plt
#from mpl_toolkits.mplot3d import Axes3D

EPSILON = 1e-5

# Triangle-triangle intersection detection based on
# "A Fast Triangle-Triangle Intersection Test" by Tomas Moller
def triangles(T1,T2):
    """
    Returns whether two triangles intersect (True) or not (False)
    Input:
        T1 - triangle defined by three vertices (3,3)
        T2 - triangle defined by three vertices (3,3)
    """
    # No intersection if all vertices in T1 are on the same side of T2's plane
    (n2,d2) = _get_plane(T2)
    d1s = np.dot(T1,n2)+d2
    _stabilize(d1s)
    if all(d1s > 0) or all(d1s < 0):
        return False

    # No intersection if all vertices in T2 are on the same side of T1's plane
    (n1,d1) = _get_plane(T1)
    d2s = np.dot(T2,n1)+d1
    _stabilize(d2s)
    if all(d2s > 0) or all(d2s < 0):
        return False

    if all(d1s == 0) and all(d2s == 0):
        # Co-planar triangles

        # Project into the coordinate plane that maximizes their area
        k = np.argmax(abs(n1)) # should be the same for n2
        mask = np.nonzero(np.array(range(3)) != k)[0]
        P1 = T1[:,mask]
        P2 = T2[:,mask]

        # Check for any edge intersections
        for L1 in it.combinations(P1,2):
            for L2 in it.combinations(P2,2):
                if lines(L1,L2):
                    return True

        # Check whether one triangle is contained within the other
        if _pt_in_triangle(P1[0],P2) or _pt_in_triangle(P2[0],P1):
            return True
        else:
            return False

    else:
        # Direction of line of intersection of the triangles
        lhat = np.cross(n1,n2)
        _stabilize(lhat)
        k = np.argmax(abs(lhat)) # find the coordinate axis most aligned with the line

        lo,hi = [],[]
        for (ds,T) in [(d1s,T1),(d2s,T2)]:
            ps = T[:,k] # project vertices onto the axis

            # Make k1 the index of the vertex with a different sign than the others
            k0,k1,k2 = 0,1,2
            if np.sign(ds[0]) == np.sign(ds[1]):
                k1,k2 = k2,k1
            elif np.sign(ds[1]) == np.sign(ds[2]):
                k1,k0 = k0,k1

            # The interval the triangle occupies on the axis
            i_a = ps[k0] + (ps[k1] - ps[k0])*ds[k0]/(ds[k0] - ds[k1])
            i_b = ps[k2] + (ps[k1] - ps[k2])*ds[k2]/(ds[k2] - ds[k1])
            lo.append(min(i_a,i_b))
            hi.append(max(i_a,i_b))

        if (lo[0] <= hi[1]) and (lo[1] <= hi[0]):
            return True
        else:
            return False


# Algorithm from the post by Gareth Rees (Feb 19 '09) on
# http://stackoverflow.com/questions/563198/how-do-you-detect-where-two-line-segments-intersect
def lines(L1,L2):
    """
    Returns whether two line segments intersect (True) or not (False)
    Input:
        L1 - line segment defined by two endpoints (2,2)
        L2 - line segment defined by two endpoints (2,2)
    """
    p = L1[0]
    r = L1[1] - L1[0]
    q = L2[0]
    s = L2[1] - L2[0]

    numer = np.cross(q-p,r)
    denom = np.cross(r,s)
    if (numer == 0) and (denom == 0):
        # Both lie in the same line
        t0 = np.dot(q-p,r)/np.dot(r,r)
        t1 = t0 + np.dot(s,r)/np.dot(r,r)
        lo = min(t0,t1)
        hi = max(t0,t1)
        if (lo <= 1) and (0 <= hi):
            return True
    elif (denom != 0):
        # Not parallel
        u = numer/denom
        t = np.cross(q-p,s)/denom
        if (0 <= t and t <= 1) and (0 <= u and u <= 1):
            return True
    # Otherwise parallel and non-intersecting
    return False

def _pt_in_triangle(p,T):
    # Express p-T[0] as a weighted sum of T[1]-T[0] and T[2]-T[0] and solve for the weights (w)
    w = np.dot(np.linalg.inv(np.transpose(T[[1,2]]-T[0])),p-T[0])
    if all(0 <= w) and all(w <= 1) and sum(w) <= 1:
        return True
    else:
        return False

def _get_plane(T):
    # T: triangle consisting of three vertices T[0],T[1],T[2]
    n = np.cross(T[1]-T[0], T[2]-T[0]) # triangle plane normal
    d = -np.dot(n,T[0]) # distance of the plane from the origin in the normal direction
    return (n,d)

def _stabilize(arr):
    # Zero all elements of arr that are less than EPSILON for numerical stability
    for i in range(len(arr)):
        if abs(arr[i]) < EPSILON:
            arr[i] = 0

def test():
    _test_pt_in_triangle()
    _test_line()
    _test_triangle()

def _test_triangle():
    # Triangle-triangle intersection (3D)
    T1 = np.array([[1.0,0,0],[0,1,0],[0,0,0.5]])
    T2 = np.array([[0.0,0,0],[1,1,0.2],[0.3,0.3,-1]])
    T3 = np.array([[0.0,0,0],[1,1,-0.2],[0.2,0.2,-2]])
    T4 = np.array([[2.0,0.2,0.2],[-2,0.2,0.2],[-2,2,2]])
    T5 = np.array([[0.0,0,0.1],[1,1,0.3],[0,0,2]])
    T6 = np.array([[2.0,0,0],[0,2,0],[0,0,1]])

#   fig = plt.figure()
#   ax = fig.add_subplot(111,projection='3d')
#   ax.plot(T1[[0,1,2,0],0],T1[[0,1,2,0],1],T1[[0,1,2,0],2],'b')
#   ax.plot(T2[[0,1,2,0],0],T2[[0,1,2,0],1],T2[[0,1,2,0],2],'g')
#   ax.plot(T3[[0,1,2,0],0],T3[[0,1,2,0],1],T3[[0,1,2,0],2],'r')
#   ax.plot(T4[[0,1,2,0],0],T4[[0,1,2,0],1],T4[[0,1,2,0],2],'c')
#   ax.plot(T5[[0,1,2,0],0],T5[[0,1,2,0],1],T5[[0,1,2,0],2],'m')
#   ax.plot(T6[[0,1,2,0],0],T6[[0,1,2,0],1],T6[[0,1,2,0],2],'y')
#   plt.show()

    npt.assert_equal(triangles(T1,T1),True)
    npt.assert_equal(triangles(T1,T2),True)
    npt.assert_equal(triangles(T1,T3),False)
    npt.assert_equal(triangles(T1,T4),True)
    npt.assert_equal(triangles(T1,T5),True)
    npt.assert_equal(triangles(T1,T6),False)

    npt.assert_equal(triangles(T2,T3),True)
    npt.assert_equal(triangles(T2,T4),False)
    npt.assert_equal(triangles(T2,T5),False)
    npt.assert_equal(triangles(T2,T6),True)

    npt.assert_equal(triangles(T3,T4),False)
    npt.assert_equal(triangles(T3,T5),False)
    npt.assert_equal(triangles(T3,T6),False)

    npt.assert_equal(triangles(T4,T5),True)
    npt.assert_equal(triangles(T4,T6),True)

    npt.assert_equal(triangles(T5,T6),True)

    arr = [T1,T2,T3,T4,T5,T6]
    for i in arr:
        for j in arr:
            npt.assert_equal(triangles(i,j),triangles(j,i))

def _test_line():
    # Line-line intersection (2D)
    L1 = np.array([[-2.0,0],[2,2]])
    L2 = np.array([[-1.0,2],[3,0]])
    L3 = np.array([[4.0,3],[6,4]])
    L4 = np.array([[0.0,-2],[0,4]])
    L5 = np.array([[-2.0,0],[0,-1]])
    L6 = np.array([[0.0,1],[1,1.5]])

    npt.assert_equal(lines(L1,L1),True)
    npt.assert_equal(lines(L1,L2),True)
    npt.assert_equal(lines(L1,L3),False)
    npt.assert_equal(lines(L1,L4),True)
    npt.assert_equal(lines(L1,L5),True)
    npt.assert_equal(lines(L1,L6),True)

    npt.assert_equal(lines(L2,L3),False)
    npt.assert_equal(lines(L2,L4),True)
    npt.assert_equal(lines(L2,L5),False)
    npt.assert_equal(lines(L2,L6),True)

    npt.assert_equal(lines(L3,L4),False)
    npt.assert_equal(lines(L3,L5),False)
    npt.assert_equal(lines(L3,L6),False)

    npt.assert_equal(lines(L4,L5),True)
    npt.assert_equal(lines(L4,L6),True)

    npt.assert_equal(lines(L5,L6),False)

    arr = [L1,L2,L3,L4,L5,L6]
    for i in arr:
        for j in arr:
            npt.assert_equal(lines(i,j),lines(j,i))

def _test_pt_in_triangle():
    # Point-triangle intersection (2D)
    T1 = np.array([[-2.0,0],[2,2],[2,-4]])
    T2 = np.array([[2.0,2],[-2,0],[2,-4]])
    T3 = np.array([[0.0,0],[1.5,0],[0,0.5]])

    npt.assert_equal(_pt_in_triangle([0.0,0],T1),True)
    npt.assert_equal(_pt_in_triangle([2.0,-2],T1),True)
    npt.assert_equal(_pt_in_triangle([-2.0,0],T1),True)
    npt.assert_equal(_pt_in_triangle([0.5,0.5],T1),True)
    npt.assert_equal(_pt_in_triangle([0.0,1.5],T1),False)
    npt.assert_equal(_pt_in_triangle([3.0,0.0],T1),False)
    npt.assert_equal(_pt_in_triangle([-2.0,-2],T1),False)
