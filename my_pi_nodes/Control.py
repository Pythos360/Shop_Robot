import math
import numpy as np


# robot geometry
# (look at pics above for explanation)
e  = 45   # end effector
f  = 80   # base
re = 272
rf = 235

# trigonometric constants
sqrt3  = math.sqrt(3.0)
pi     = 3.141592653  # PI
sin120 = sqrt3 / 2.0
cos120 = -0.5
tan60  = sqrt3
sin30  = 0.5
tan30  = 1.0 / sqrt3


# forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
# returned status: 0=OK, -1=non-existing position
def delta_calcForward(theta1: float, theta2: float, theta3: float):
    t = (f - e) * tan30 / 2.0
    dtr = pi / 180.0

    # convert to radians
    theta1 *= dtr
    theta2 *= dtr
    theta3 *= dtr

    y1 = -(t + rf * math.cos(theta1))
    z1 = -rf * math.sin(theta1)

    y2 = (t + rf * math.cos(theta2)) * sin30
    x2 = y2 * tan60
    z2 = -rf * math.sin(theta2)

    y3 = (t + rf * math.cos(theta3)) * sin30
    x3 = -y3 * tan60
    z3 = -rf * math.sin(theta3)

    dnm = (y2 - y1) * x3 - (y3 - y1) * x2

    w1 = y1 * y1 + z1 * z1
    w2 = x2 * x2 + y2 * y2 + z2 * z2
    w3 = x3 * x3 + y3 * y3 + z3 * z3

    # x = (a1*z + b1)/dnm
    a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
    b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

    # y = (a2*z + b2)/dnm
    a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
    b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

    # a*z^2 + b*z + c = 0
    a = a1 * a1 + a2 * a2 + dnm * dnm
    b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
    c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re)

    # discriminant
    d = b * b - 4.0 * a * c
    if d < 0.0:
        return -1, None, None, None  # non-existing point

    z0 = -0.5 * (b + math.sqrt(d)) / a
    x0 = (a1 * z0 + b1) / dnm
    y0 = (a2 * z0 + b2) / dnm
    return 0, x0, y0, z0


# inverse kinematics
# helper function, calculates angle theta (for YZ-plane)
# returned status: 0=OK, -1=non-existing position
def delta_calcAngleYZ(x0: float, y0: float, z0: float):
    y1 = -0.5 * 0.57735 * f      # f/2 * tg 30
    y0 = y0 - 0.5 * 0.57735 * e  # shift center to edge

    # z = a + b*y
    a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0)
    b = (y1 - y0) / z0

    # discriminant
    d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
    if d < 0.0:
        return -1, None  # non-existing point

    yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1.0)  # choosing outer point
    zj = a + b * yj

    theta = 180.0 * math.atan(-zj / (y1 - yj)) / pi + (180.0 if yj > y1 else 0.0)
    return 0, theta


# inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
# returned status: 0=OK, -1=non-existing position
def delta_calcInverse(x0: float, y0: float, z0: float):
    theta1 = theta2 = theta3 = 0.0

    status, theta1 = delta_calcAngleYZ(x0, y0, z0)
    if status == 0:
        status, theta2 = delta_calcAngleYZ(
            x0 * cos120 + y0 * sin120,
            y0 * cos120 - x0 * sin120,
            z0
        )  # rotate coords to +120 deg
    if status == 0:
        status, theta3 = delta_calcAngleYZ(
            x0 * cos120 - y0 * sin120,
            y0 * cos120 + x0 * sin120,
            z0
        )  # rotate coords to -120 deg

    if status != 0:
        return -1, None, None, None

    return 0, theta1, theta2, theta3

class robotmodel:

    def __init__(self, f: float, e: float, re: float, rf:float):

        sqrt3  = math.sqrt(3.0)
        pi     = 3.141592653  # PI
        sin120 = sqrt3 / 2.0
        cos120 = -0.5
        tan60  = sqrt3
        sin30  = 0.5
        tan30  = 1.0 / sqrt3

    # forward kinematics: (theta1, theta2, theta3) -> (x0, y0, z0)
# returned status: 0=OK, -1=non-existing position
    def delta_calcForward(self, theta1: float, theta2: float, theta3: float):
        t = (f - e) * tan30 / 2.0
        dtr = pi / 180.0

        # convert to radians
        theta1 *= dtr
        theta2 *= dtr
        theta3 *= dtr

        y1 = -(t + rf * math.cos(theta1))
        z1 = -rf * math.sin(theta1)

        y2 = (t + rf * math.cos(theta2)) * sin30
        x2 = y2 * tan60
        z2 = -rf * math.sin(theta2)

        y3 = (t + rf * math.cos(theta3)) * sin30
        x3 = -y3 * tan60
        z3 = -rf * math.sin(theta3)

        dnm = (y2 - y1) * x3 - (y3 - y1) * x2

        w1 = y1 * y1 + z1 * z1
        w2 = x2 * x2 + y2 * y2 + z2 * z2
        w3 = x3 * x3 + y3 * y3 + z3 * z3

        # x = (a1*z + b1)/dnm
        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

        # y = (a2*z + b2)/dnm
        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        # a*z^2 + b*z + c = 0
        a = a1 * a1 + a2 * a2 + dnm * dnm
        b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
        c = (b2 - y1 * dnm) * (b2 - y1 * dnm) + b1 * b1 + dnm * dnm * (z1 * z1 - re * re)

        # discriminant
        d = b * b - 4.0 * a * c
        if d < 0.0:
            return -1, None, None, None  # non-existing point

        z0 = -0.5 * (b + math.sqrt(d)) / a
        x0 = (a1 * z0 + b1) / dnm
        y0 = (a2 * z0 + b2) / dnm
        return np.array([x0, y0, z0], dtype = float)


    # inverse kinematics
    # helper function, calculates angle theta (for YZ-plane)
    # returned status: 0=OK, -1=non-existing position
    def delta_calcAngleYZ(selfm, x0: float, y0: float, z0: float):
        y1 = -0.5 * 0.57735 * f      # f/2 * tg 30
        y0 = y0 - 0.5 * 0.57735 * e  # shift center to edge

        # z = a + b*y
        a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0)
        b = (y1 - y0) / z0

        # discriminant
        d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
        if d < 0.0:
            return -1, None  # non-existing point

        yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1.0)  # choosing outer point
        zj = a + b * yj

        theta = 180.0 * math.atan(-zj / (y1 - yj)) / pi + (180.0 if yj > y1 else 0.0)
        return 0, theta


    # inverse kinematics: (x0, y0, z0) -> (theta1, theta2, theta3)
    # returned status: 0=OK, -1=non-existing position
    def delta_calcInverse(self, x0: float, y0: float, z0: float):
        theta1 = theta2 = theta3 = 0.0

        status, theta1 = delta_calcAngleYZ(x0, y0, z0)
        if status == 0:
            status, theta2 = delta_calcAngleYZ(
                x0 * cos120 + y0 * sin120,
                y0 * cos120 - x0 * sin120,
                z0
            )  # rotate coords to +120 deg
        if status == 0:
            status, theta3 = delta_calcAngleYZ(
                x0 * cos120 - y0 * sin120,
                y0 * cos120 + x0 * sin120,
                z0
            )  # rotate coords to -120 deg

        if status != 0:
            return -1, None, None, None

        return 0, theta1, theta2, theta3
    

class dynamics:
    def __init__(self, thetas_init):
        self.thetas = np.array(thetas_init, dtype=float)  # deg
        self.J = np.zeros((3, 3), dtype=float)
        # [x_min, x_max], [y_min, y_max], [z_min, z_max] in mm
        self.limits = np.array([
            [-160.8, 160.8],
            [-160.8, 160.8],
            [-370.0, -100.0],
        ])

    def fk(self, thetas_deg):
        t = np.asarray(thetas_deg, dtype=float)
        status, x0, y0, z0 = delta_calcForward(t[0], t[1], t[2])
        if status != 0:
            raise ValueError(f"FK failed for {t}")
        return np.array([x0, y0, z0], dtype=float)

    def position(self):
        """Current tip position [x,y,z] in mm."""
        return self.fk(self.thetas)

    def numJ(self, h=0.1):
        theta = np.array(self.thetas, dtype=float)
        for i in range(3):
            dtheta = np.zeros(3)
            dtheta[i] = h
            x_plus  = self.fk(theta + dtheta)
            x_minus = self.fk(theta - dtheta)
            self.J[:, i] = (x_plus - x_minus) / (2.0 * h)
        return self.J

    def qdot_from_v(self, v, gain=200):
        v = np.asarray(v, dtype=float)

        # Deadzone
        v[np.abs(v) < 0.1] = 0.0

        # Scale joystick to mm/s
        v = v * gain

        # --- Workspace limiting in Cartesian space ---
        pos = self.position()  # [x,y,z]
        print(pos)
        for i in range(3):     # 0:x, 1:y, 2:z
            lower, upper = self.limits[i]
            span = upper - lower
            if span <= 0:
                continue

            # If we're close to upper bound and commanding + velocity, zero that axis
            if v[i] > 0 and (upper - pos[i]) < 0.05 * span:
                v[i] = 0.0
            # If we're close to lower bound and commanding - velocity, zero that axis
            if v[i] < 0 and (pos[i] - lower) < 0.05 * span:
                v[i] = 0.0

        # --- Map remaining Cartesian velocity to joint velocities ---
        J = self.numJ()
        qd = np.linalg.pinv(J) @ v  # deg/s

        return qd
    
    def step(self, dt, gain=.0000005):
        qd = self.qdot(gain=gain)   # deg/s
        self.thetas = self.thetas + qd * dt
        return self.thetas
    
    def position(self):
        """Convenience: current tip position [x,y,z] in mm."""
        return self.fk(self.thetas)
    
    
