import modern_robotics as mr
from math import sqrt,cos,sin,pi
import numpy as np


def changePointScrew(s_w,s_v,p):
    """ Function to define the screw axis in an other point
    Parameters :
        - s_w : rotation in point A
        - s_v : translation in point A
        - p : new point
    """
    s_v = s_v - np.dot(mr.VecToso3(s_w),p)
    s = np.concatenate((s_w,s_v))
    return s

def point2Homogenous(point,m):
    t = m.copy()
    for i in range(3):
        t[i][3] = point[i]
    return t
