import numpy as np

def quat_mul(q1,q2):
    r = np.concatenate((q1[0:1]*q2[0:1] - np.dot(q1[1:4],q2[1:4]),q1[0:1]*q2[1:4] + q2[0:1]*q1[1:4] + np.cross(q1[1:4],q2[1:4])),axis = 0)
    r = quat_norm(r)
    return r

def quat_conj(q1):
    r = np.concatenate((q1[0:1],-1*q1[1:4]),axis = 0)
    r = quat_norm(r)
    #r = -1*q1[1:4]
    return r

def quat_mul3(q1,q2,q3):
    r = quat_mul(quat_mul(q1,q2),q3)
    return r

def quat_norm(q1):
    norm = np.linalg.norm(q1)
    if norm == 0:
        r = q1
    else:
        r = q1/norm
    return r