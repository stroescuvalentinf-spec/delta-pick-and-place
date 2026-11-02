# venv/kinematics.py
"""
Kinematics helper functions for the Delta robot:
- delta_calcAngleYZ
- delta_calcInverse
- delta_calcForward
- compute_elbow_positions
- ee_connector_points_rigid

All units: mm / degrees
"""
import math
import numpy as np

sqrt3 = math.sqrt(3.0)
pi = math.pi
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60 = sqrt3
sin30 = 0.5
tan30 = 1.0 / tan60


def delta_calcAngleYZ(x0, y0, z0, e, f, re, rf, min_alpha_deg=-90.0, max_alpha_deg=90.0):
    """
    Calculates the motor angle for one arm.
    Also calculates the upper arm angle (alpha) and checks it against limits.
    Returns: (status, motor_angle, upper_arm_angle)
    status: 0 for success, -1 for non-existing point, -2 for angle limit violation
    """
    y1 = -0.5 * 0.57735 * f
    y0p = y0 - 0.5 * 0.57735 * e

    # Check for singularity when z0 is close to zero
    if abs(z0) < 1e-6:
        return -1, None, None

    a = (x0 * x0 + y0p * y0p + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0)
    b = (y1 - y0p) / z0
    d = -(a + b * y1) * (a + b * y1) + rf * rf * (b * b + 1.0)
    if d < 0:
        return -1, None, None

    yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1.0)
    zj = a + b * yj

    # Calculate motor angle (theta)
    theta = math.degrees(math.atan2(-zj, y1 - yj)) if (y1 - yj) != 0 else (90 if -zj > 0 else -90)

    # *** FIX: Corrected alpha angle calculation ***
    # Convention: 0 is horizontal, positive is down, negative is up.
    # Vertical component is -zj because robot Z is negative downwards.
    # Horizontal component is (y1 - yj) which is the projection length.
    alpha_rad = math.atan2(-zj, y1 - yj)
    alpha_deg = math.degrees(alpha_rad)

    # Check if alpha is within the specified limits
    if not (min_alpha_deg <= alpha_deg <= max_alpha_deg):
        return -2, theta, alpha_deg  # Angle limit violated

    return 0, theta, alpha_deg


def delta_calcInverse(x0, y0, z0, e, f, re, rf, min_alpha_deg=-90.0, max_alpha_deg=90.0):
    """
    Calculates inverse kinematics for all three motors.
    Returns: (status, t1, t2, t3)
    status: 0=OK, -1=unreachable, -2=angle_limit_violation
    """
    # Tower 1
    status1, t1, _ = delta_calcAngleYZ(x0, y0, z0, e, f, re, rf, min_alpha_deg, max_alpha_deg)
    if status1 != 0:
        return status1, None, None, None

    # Tower 2
    status2, t2, _ = delta_calcAngleYZ(x0 * cos120 + y0 * sin120,
                                       y0 * cos120 - x0 * sin120,
                                       z0, e, f, re, rf, min_alpha_deg, max_alpha_deg)
    if status2 != 0:
        return status2, None, None, None

    # Tower 3
    status3, t3, _ = delta_calcAngleYZ(x0 * cos120 - y0 * sin120,
                                       y0 * cos120 + x0 * sin120,
                                       z0, e, f, re, rf, min_alpha_deg, max_alpha_deg)
    if status3 != 0:
        return status3, None, None, None

    return 0, t1, t2, t3


def delta_calcForward(theta1, theta2, theta3, e, f, re, rf):
    t = (f - e) * tan30 / 2.0
    dtr = pi / 180.0
    th1 = theta1 * dtr
    th2 = theta2 * dtr
    th3 = theta3 * dtr

    y1 = -(t + rf * math.cos(th1))
    z1 = -rf * math.sin(th1)

    y2 = (t + rf * math.cos(th2)) * sin30
    x2 = y2 * tan60
    z2 = -rf * math.sin(th2)

    y3 = (t + rf * math.cos(th3)) * sin30
    x3 = -y3 * tan60
    z3 = -rf * math.sin(th3)

    dnm = (y2 - y1) * x3 - (y3 - y1) * x2
    if abs(dnm) < 1e-9: return -1, None, None, None

    w1 = y1 * y1 + z1 * z1
    w2 = x2 * x2 + y2 * y2 + z2 * z2
    w3 = x3 * x3 + y3 * y3 + z3 * z3

    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0
    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

    a = a1 * a1 + a2 * a2 + dnm * dnm
    b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
    c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re)

    d = b * b - 4.0 * a * c
    if d < 0 or abs(a) < 1e-9: return -1, None, None, None

    z0 = -0.5 * (b + math.sqrt(d)) / a
    x0 = (a1 * z0 + b1) / dnm
    y0 = (a2 * z0 + b2) / dnm
    return 0, x0, y0, z0


def compute_elbow_positions(theta1, theta2, theta3, f, rf):
    R = (f * 0.57735) / 2.0
    v1 = np.array([0.0, -R, 0.0])

    def rz(a_deg):
        a = math.radians(a_deg)
        return np.array([[math.cos(a), -math.sin(a), 0.0],
                         [math.sin(a), math.cos(a), 0.0],
                         [0.0, 0.0, 1.0]])

    R120 = rz(120);
    Rm120 = rz(-120)
    base = np.vstack([v1, R120.dot(v1), Rm120.dot(v1)])
    dtr = math.pi / 180.0

    def local_elbow(theta_deg):
        th = theta_deg * dtr
        return np.array([0.0, -rf * math.cos(th), -rf * math.sin(th)])

    e1 = base[0] + local_elbow(theta1)
    e2 = base[1] + R120.dot(local_elbow(theta2))
    e3 = base[2] + Rm120.dot(local_elbow(theta3))
    return np.vstack([e1, e2, e3]), base


def ee_connector_points_rigid(center_x, center_y, center_z, e):
    shift = 0.5 * tan30 * e
    local = np.array([0.0, -shift, 0.0])

    def rz(a_deg):
        a = math.radians(a_deg)
        return np.array([[math.cos(a), -math.sin(a), 0.0],
                         [math.sin(a), math.cos(a), 0.0],
                         [0.0, 0.0, 1.0]])

    c = np.array([center_x, center_y, center_z])
    p1 = c + rz(0).dot(local)
    p2 = c + rz(120).dot(local)
    p3 = c + rz(-120).dot(local)
    return np.vstack([p1, p2, p3])