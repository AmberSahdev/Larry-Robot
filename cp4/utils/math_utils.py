import numpy as np
from math import cos, sin
from scipy.linalg import expm,logm

# returns the skew matrix of a 3x1 vector
def skew(w):
    skew_w = np.zeros((3,3))
    skew_w[0, 1] = -w[2]
    skew_w[0, 2] = w[1]
    skew_w[1, 0] = w[2]
    skew_w[1, 2] = -w[0]
    skew_w[2, 0] = -w[1]
    skew_w[2, 1] = w[0]
    return skew_w

# returns the skew matrix of 6x1 vector or Twist
def skew6(S):
    w = S[:3]
    v = S[3:]
    skew_S = np.zeros((4, 4))
    skew_w = skew(w)
    skew_S[:3, :3] = skew_w
    skew_S[:3, 3] = v.reshape((1,3))
    return skew_S

# returns the adjoint of transformation matrix T
def adjoint(T):
    R = T[:3, :3]
    p = T[:3, 3]

    skew_p = skew(p)
    skew_p_times_R = np.matmul(skew_p, R)

    adjT = np.zeros((6,6))
    adjT[:3, :3] = R
    adjT[3:, :3] = skew_p_times_R
    adjT[3:, 3:] = R
    return adjT
