# Control_Test.py
import math
import numpy as np

# -----------------------------
# Robot geometry (mm)
# -----------------------------
E = 45.0
F = 80.0
RE = 272.0
RF = 235.0

# -----------------------------
# Trig constants
# -----------------------------
SQRT3 = math.sqrt(3.0)
PI = math.pi

SIN120 = SQRT3 / 2.0
COS120 = -0.5
TAN60 = SQRT3
SIN30 = 0.5
TAN30 = 1.0 / SQRT3


class RobotModel:
    """
    Delta robot kinematics model (classic delta geometry).

    Angles are in degrees.
    Positions are in mm.
    """

    def __init__(self, f=F, e=E, re=RE, rf=RF):
        self.f = float(f)
        self.e = float(e)
        self.re = float(re)
        self.rf = float(rf)

    def delta_calcForward(self, theta1: float, theta2: float, theta3: float):
        """Forward kinematics: (theta1,theta2,theta3 deg) -> (status,x,y,z mm)."""
        f, e, re, rf = self.f, self.e, self.re, self.rf

        t = (f - e) * TAN30 / 2.0
        dtr = PI / 180.0

        theta1 *= dtr
        theta2 *= dtr
        theta3 *= dtr

        y1 = -(t + rf * math.cos(theta1))
        z1 = -rf * math.sin(theta1)

        y2 = (t + rf * math.cos(theta2)) * SIN30
        x2 = y2 * TAN60
        z2 = -rf * math.sin(theta2)

        y3 = (t + rf * math.cos(theta3)) * SIN30
        x3 = -y3 * TAN60
        z3 = -rf * math.sin(theta3)

        dnm = (y2 - y1) * x3 - (y3 - y1) * x2

        w1 = y1 * y1 + z1 * z1
        w2 = x2 * x2 + y2 * y2 + z2 * z2
        w3 = x3 * x3 + y3 * y3 + z3 * z3

        a1 = (z2 - z1) * (y3 - y1) - (z3 - z1) * (y2 - y1)
        b1 = -((w2 - w1) * (y3 - y1) - (w3 - w1) * (y2 - y1)) / 2.0

        a2 = -(z2 - z1) * x3 + (z3 - z1) * x2
        b2 = ((w2 - w1) * x3 - (w3 - w1) * x2) / 2.0

        a = a1 * a1 + a2 * a2 + dnm * dnm
        b = 2.0 * (a1 * b1 + a2 * (b2 - y1 * dnm) - z1 * dnm * dnm)
        c = (b2 - y1 * dnm) ** 2 + b1 * b1 + dnm * dnm * (z1 * z1 - re * re)

        d = b * b - 4.0 * a * c
        if d < 0.0:
            return -1, None, None, None

        z0 = -0.5 * (b + math.sqrt(d)) / a
        x0 = (a1 * z0 + b1) / dnm
        y0 = (a2 * z0 + b2) / dnm

        return 0, x0, y0, z0

    def delta_calcAngleYZ(self, x0: float, y0: float, z0: float):
        """Helper for inverse kinematics."""
        f, e, re, rf = self.f, self.e, self.re, self.rf

        y1 = -0.5 * 0.57735 * f
        y0 = y0 - 0.5 * 0.57735 * e

        a = (x0 * x0 + y0 * y0 + z0 * z0 + rf * rf - re * re - y1 * y1) / (2.0 * z0)
        b = (y1 - y0) / z0

        d = -(a + b * y1) * (a + b * y1) + rf * (b * b * rf + rf)
        if d < 0.0:
            return -1, None

        yj = (y1 - a * b - math.sqrt(d)) / (b * b + 1.0)
        zj = a + b * yj

        theta_rad = math.atan2(-zj, y1 - yj)
        theta = 180.0 * theta_rad / PI

        if theta < -180.0:
            theta += 360.0
        elif theta > 180.0:
            theta -= 360.0

        return 0, theta

    def delta_calcInverse(self, x0: float, y0: float, z0: float):
        """Inverse kinematics: (x,y,z mm) -> (status, theta1, theta2, theta3 deg)."""
        status, t1 = self.delta_calcAngleYZ(x0, y0, z0)
        if status != 0:
            return -1, None, None, None

        status, t2 = self.delta_calcAngleYZ(
            x0 * COS120 + y0 * SIN120,
            y0 * COS120 - x0 * SIN120,
            z0,
        )
        if status != 0:
            return -1, None, None, None

        status, t3 = self.delta_calcAngleYZ(
            x0 * COS120 - y0 * SIN120,
            y0 * COS120 + x0 * SIN120,
            z0,
        )
        if status != 0:
            return -1, None, None, None

        return 0, t1, t2, t3


class dynamics:
    """
    Minimal dynamics wrapper:
      - maintains joint state (deg)
      - FK
      - numerical Jacobian
      - qdot_from_v (DLS)
      - step() returns (thetas, qdot)
    """

    def __init__(self, thetas_init, model: RobotModel | None = None):
        self.thetas = np.array(thetas_init, dtype=float)
        self.J = np.zeros((3, 3), dtype=float)
        self.model = model if model is not None else RobotModel()

    def fk(self, thetas_deg):
        t = np.asarray(thetas_deg, dtype=float)
        status, x0, y0, z0 = self.model.delta_calcForward(t[0], t[1], t[2])
        if status != 0:
            raise ValueError(f"FK failed for {t}")
        return np.array([x0, y0, z0], dtype=float)

    def position(self):
        return self.fk(self.thetas)

    def numJ(self, h=0.1):
        theta = np.array(self.thetas, dtype=float)
        for i in range(3):
            dtheta = np.zeros(3)
            dtheta[i] = h
            x_plus = self.fk(theta + dtheta)
            x_minus = self.fk(theta - dtheta)
            self.J[:, i] = (x_plus - x_minus) / (2.0 * h)
        return self.J

    def qdot_from_v(self, v, gain=5.0, deadzone=0.1):
        """
        v: tip command vector (unitless or mm/s depending on your calling convention)
        gain: scales v -> mm/s
        returns qdot in deg/s
        """
        v = np.asarray(v, dtype=float).copy()

        if deadzone and deadzone > 0.0:
            v[np.abs(v) < deadzone] = 0.0

        v = v * float(gain)

        J = self.numJ()
        _, S, _ = np.linalg.svd(J)
        if S[-1] < 1e-6:
            return np.zeros(3, dtype=float)

        condJ = S[0] / S[-1]

        lam_base = 0.5
        if condJ > 20.0:
            lam = 2.0 + 0.1 * (condJ - 20.0)
        else:
            lam = lam_base * (1.0 + max(0.0, (condJ - 5.0) / 5.0))

        JT = J.T
        qdot = JT @ np.linalg.inv(J @ JT + (lam ** 2) * np.eye(3)) @ v

        if condJ > 15.0:
            vel_scale = 1.0 / (1.0 + 0.1 * (condJ - 15.0))
            qdot = qdot * vel_scale

        return qdot

    def step(self, dt, v=None, gain=5.0, deadzone=0.1):
        """
        Integrate one timestep.
        Returns (thetas, qdot) with:
          thetas: deg
          qdot: deg/s
        """
        vel = np.zeros(3, dtype=float) if v is None else np.asarray(v, dtype=float)
        qdot = self.qdot_from_v(vel, gain=gain, deadzone=deadzone)

        self.thetas = self.thetas + qdot * float(dt)
        return self.thetas.copy(), qdot


# -----------------------------
# Physical motor mapping helper
# -----------------------------
def qdot_to_off_us_physical(
    qdot_deg_s,
    deg_per_step=0.045,
    on_us=5.0,
    off_us_min=1000.0,
    off_us_max=100000.0,
    qdot_stop=0.05,
):
    """
    Convert qdot (deg/s) -> signed off_us for Arduino scheduler.

    Arduino behavior:
      - step pulse HIGH for on_us
      - step pulse LOW for off_us
      - 0 means stop
      - sign gives direction

    For desired speed:
      steps/s = |qdot| / deg_per_step
      period_us = 1e6 / steps/s
      off_us = period_us - on_us

    qdot_stop prevents tiny solver noise from becoming "slow motion".
    """
    qdot = np.asarray(qdot_deg_s, dtype=float)
    out = np.zeros(3, dtype=float)

    for i, qd in enumerate(qdot):
        if abs(qd) < qdot_stop:
            out[i] = 0.0
            continue

        steps_per_s = abs(qd) / float(deg_per_step)
        if steps_per_s < 1e-9:
            out[i] = 0.0
            continue

        period_us = 1e6 / steps_per_s
        off_us = period_us - float(on_us)
        off_us = float(np.clip(off_us, off_us_min, off_us_max))

        out[i] = math.copysign(off_us, qd)

    return out
