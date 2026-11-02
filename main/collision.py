"""
Collision utilities: segment-segment shortest distance and pose collision checks
for the Delta robot link geometry.
"""
import numpy as np
from kinematics import delta_calcInverse, compute_elbow_positions, ee_connector_points_rigid

def seg_seg_distance(p1, q1, p2, q2):
    p1 = np.array(p1, dtype=float)
    q1 = np.array(q1, dtype=float)
    p2 = np.array(p2, dtype=float)
    q2 = np.array(q2, dtype=float)
    u = q1 - p1
    v = q2 - p2
    w0 = p1 - p2
    a = np.dot(u,u)
    b = np.dot(u,v)
    c = np.dot(v,v)
    d = np.dot(u,w0)
    e = np.dot(v,w0)
    D = a*c - b*b
    SMALL = 1e-12
    sN = 0.0; sD = D
    tN = 0.0; tD = D

    if D < SMALL:
        sN = 0.0
        sD = 1.0
        tN = e
        tD = c
    else:
        sN = (b*e - c*d)
        tN = (a*e - b*d)
        if sN < 0.0:
            sN = 0.0
            tN = e
            tD = c
        elif sN > sD:
            sN = sD
            tN = e + b
            tD = c

    if tN < 0.0:
        tN = 0.0
        if -d < 0.0:
            sN = 0.0
        elif -d > a:
            sN = sD
        else:
            sN = -d
            sD = a
    elif tN > tD:
        tN = tD
        if (-d + b) < 0.0:
            sN = 0.0
        elif (-d + b) > a:
            sN = sD
        else:
            sN = (-d + b)
            sD = a

    sc = 0.0 if abs(sN) < SMALL else sN / sD
    tc = 0.0 if abs(tN) < SMALL else tN / tD

    dP = w0 + (sc * u) - (tc * v)
    return np.linalg.norm(dP)

def check_pose_self_collision(x, y, z, e, f, re, rf, clearance=0.5, upper_radius=8.0, lower_radius=4.5):
    st, th1, th2, th3 = delta_calcInverse(x, y, z, e, f, re, rf)
    if st != 0:
        # unreachable - consider as collision/invalid for safety
        return True
    elbows, base = compute_elbow_positions(th1, th2, th3, f, rf)
    ee_conn = ee_connector_points_rigid(x, y, z, e)
    upper_segs = [(base[i], elbows[i]) for i in range(3)]
    lower_segs = [(elbows[i], ee_conn[i]) for i in range(3)]

    # upper-upper collisions
    for i in range(3):
        for j in range(i+1, 3):
            d = seg_seg_distance(upper_segs[i][0], upper_segs[i][1], upper_segs[j][0], upper_segs[j][1])
            if d < (upper_radius + upper_radius + clearance):
                return True
    # lower-lower collisions
    for i in range(3):
        for j in range(i+1, 3):
            d = seg_seg_distance(lower_segs[i][0], lower_segs[i][1], lower_segs[j][0], lower_segs[j][1])
            if d < (lower_radius + lower_radius + clearance):
                return True
    # upper vs other lowers (i != j)
    for i in range(3):
        for j in range(3):
            if i == j:
                continue
            d = seg_seg_distance(upper_segs[i][0], upper_segs[i][1], lower_segs[j][0], lower_segs[j][1])
            if d < (upper_radius + lower_radius + clearance):
                return True
    return False
