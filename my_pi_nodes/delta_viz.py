# delta_viz_node.py
import math, time
import numpy as np
from numpy import cos, sin
from dataclasses import dataclass
from typing import Iterable, Tuple, List

import matplotlib
# Use an interactive backend so draw calls don’t block:
matplotlib.use("Qt5Agg")  # or "TkAgg" if you prefer
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32MultiArray  # optional: for motor steps

# ---------- Pure model/visualization (unchanged in spirit) ----------

@dataclass
class DeltaParams:
    base_radius: float = 0.18
    plat_radius: float = 0.055
    upper_len: float  = 0.22
    lower_len: float  = 0.42

def tri_vertices(radius: float) -> np.ndarray:
    ang = np.array([0, 2*np.pi/3, 4*np.pi/3])
    return np.c_[radius*np.cos(ang), radius*np.sin(ang), np.zeros(3)]

def leg_frames(radius: float) -> List[np.ndarray]:
    ang = np.array([0, 2*np.pi/3, 4*np.pi/3])
    frames = []
    for a in ang:
        R = np.array([[cos(a), -sin(a), 0],
                      [sin(a),  cos(a), 0],
                      [0,       0,      1]])
        frames.append(R)
    return frames

def elbows_from_angles(params: DeltaParams, thetas: Iterable[float]) -> np.ndarray:
    B = tri_vertices(params.base_radius)
    Rlegs = leg_frames(params.base_radius)
    E = np.zeros((3,3))
    for i, th in enumerate(thetas):
        v_local = np.array([params.upper_len*cos(th), 0.0, -params.upper_len*sin(th)])
        v_world = Rlegs[i] @ v_local
        E[i] = B[i] + v_world
    return E

def platform_points(params: DeltaParams, center: np.ndarray) -> np.ndarray:
    return tri_vertices(params.plat_radius) + center

def fk_objective(xyz: np.ndarray, params: DeltaParams, E: np.ndarray) -> np.ndarray:
    P = platform_points(params, xyz)
    return np.linalg.norm(P - E, axis=1) - params.lower_len

def solve_fk(params: DeltaParams, thetas: Iterable[float], z_guess: float = -0.3):
    # Tiny, local numeric solve; SciPy is nice, but we’ll do a vanilla Newton-like loop
    # to avoid SciPy dependency in your ROS image.
    E = elbows_from_angles(params, thetas)
    C = np.array([0.0, 0.0, z_guess])
    for _ in range(30):  # few iterations are plenty
        P = platform_points(params, C)
        d = np.linalg.norm(P - E, axis=1)
        r = d - params.lower_len                            # residuals
        if np.max(np.abs(r)) < 1e-6:
            break
        # simple Jacobian wrt (x,y,z): unit vectors from elbows to platform pts
        u = (P - E) / (d[:,None] + 1e-12)
        J = u.sum(axis=0).reshape(1,3) * 0.0  # placeholder to keep shape visible
        # For independent constraints, stack rows:
        J = u  # 3x3
        # least-squares step
        step, *_ = np.linalg.lstsq(J, -r, rcond=None)
        C = C + step
    P = platform_points(params, C)
    return E, C, P

class DeltaVisualizer:
    def __init__(self, params: DeltaParams):
        self.params = params
        plt.ion()
        self.fig = plt.figure(figsize=(7,7))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_box_aspect((1,1,0.6))
        r = params.base_radius + params.upper_len + 0.1
        self.ax.set_xlim(-r, r)
        self.ax.set_ylim(-r, r)
        self.ax.set_zlim(-(params.upper_len+params.lower_len+0.05), 0.1)
        self.ax.set_xlabel('X (m)'); self.ax.set_ylabel('Y (m)'); self.ax.set_zlabel('Z (m)')
        self.ax.view_init(elev=25, azim=45)

        B = tri_vertices(params.base_radius)
        self.ax.scatter(B[:,0], B[:,1], B[:,2], s=40)
        circ = np.linspace(0, 2*np.pi, 120)
        self.ax.plot(params.base_radius*np.cos(circ), params.base_radius*np.sin(circ), 0*circ, lw=1.0)

        self.upper_lines = [self.ax.plot([],[],[], lw=3)[0] for _ in range(3)]
        self.lower_lines = [self.ax.plot([],[],[], lw=3)[0] for _ in range(3)]
        self.elbows = self.ax.scatter([],[],[], s=25)
        self.plat = self.ax.scatter([],[],[], s=25)
        self.plat_outline, = self.ax.plot([],[],[], lw=2)

    def draw_state(self, thetas_rad: Iterable[float]):
        E, C, P = solve_fk(self.params, thetas_rad)
        B = tri_vertices(self.params.base_radius)

        for i in range(3):
            self.upper_lines[i].set_data([B[i,0], E[i,0]], [B[i,1], E[i,1]])
            self.upper_lines[i].set_3d_properties([B[i,2], E[i,2]])

            self.lower_lines[i].set_data([E[i,0], P[i,0]], [E[i,1], P[i,1]])
            self.lower_lines[i].set_3d_properties([E[i,2], P[i,2]])

        self.elbows._offsets3d = (E[:,0], E[:,1], E[:,2])
        self.plat._offsets3d   = (P[:,0], P[:,1], P[:,2])

        tri = np.vstack([P, P[0]])
        self.plat_outline.set_data(tri[:,0], tri[:,1])
        self.plat_outline.set_3d_properties(tri[:,2])

        self.ax.set_title(f"Center: ({C[0]:.3f}, {C[1]:.3f}, {C[2]:.3f}) m")
        self.fig.canvas.draw_idle()
        plt.pause(0.001)

# ---------- ROS 2 Node wrapper ----------

@dataclass
class MotorModel:
    steps_per_rev: int = 200
    microsteps: int = 16
    gear_ratio: float = 1.0
    zero_offset_deg: Tuple[float,float,float] = (0,0,0)

    def steps_to_rad(self, steps: Tuple[int,int,int]) -> np.ndarray:
        spm = self.steps_per_rev * self.microsteps * self.gear_ratio
        deg = (np.array(steps) / spm) * 360.0 + np.array(self.zero_offset_deg)
        return np.deg2rad(deg)

class DeltaVizNode(Node):
    def __init__(self):
        super().__init__('delta_viz')

        # Parameters (declared so you can set them via YAML/CLI)
        self.declare_parameter('base_radius', 0.18)
        self.declare_parameter('plat_radius', 0.055)
        self.declare_parameter('upper_len',   0.22)
        self.declare_parameter('lower_len',   0.42)
        self.declare_parameter('viz_hz',      30.0)

        self.params = DeltaParams(
            base_radius = self.get_parameter('base_radius').value,
            plat_radius = self.get_parameter('plat_radius').value,
            upper_len   = self.get_parameter('upper_len').value,
            lower_len   = self.get_parameter('lower_len').value,
        )
        self.viz = DeltaVisualizer(self.params)

        # State
        self.last_thetas = np.deg2rad(np.array([20.0, 20.0, 20.0]))
        self.last_joy_time = time.time()

        # Subscriptions
        self.sub_joy = self.create_subscription(Joy, 'joy', self.on_joy, 10)
        # Optional: direct steps input (Int32MultiArray with 3 entries)
        self.sub_steps = self.create_subscription(Int32MultiArray, 'delta/steps', self.on_steps, 10)

        # Motor model (used only if you publish to /delta/steps)
        self.model = MotorModel(steps_per_rev=200, microsteps=16, gear_ratio=1.0,
                                zero_offset_deg=(0,0,0))

        # Timer to draw at viz_hz
        hz = float(self.get_parameter('viz_hz').value)
        self.timer = self.create_timer(1.0/max(hz, 1.0), self.on_timer)

        self.get_logger().info('delta_viz node ready. Subscribed to /joy and /delta/steps')

    # --- Callbacks ---

    def on_joy(self, msg: Joy):
        """
        Map joystick axes to *incremental* joint angle changes.
        Example: left stick X->θ1, left stick Y->θ2, right stick X->θ3
        """
        now = time.time()
        dt = max(1e-3, now - self.last_joy_time)
        self.last_joy_time = now

        # Defensive: ensure axes exist
        ax = list(msg.axes) + [0.0, 0.0, 0.0]
        a1, a2, a3 = ax[0], ax[1], ax[3]

        # Tunables
        max_rate = np.deg2rad(45.0)  # rad/s per full stick deflection
        dtheta = max_rate * dt * np.array([a1, a2, a3])
        self.last_thetas = self.last_thetas + dtheta

        # Clamp if you want (example: 0..75 deg)
        lo, hi = np.deg2rad(0.0), np.deg2rad(75.0)
        self.last_thetas = np.clip(self.last_thetas, lo, hi)

    def on_steps(self, msg: Int32MultiArray):
        data = list(msg.data)
        if len(data) < 3:
            self.get_logger().warn('delta/steps needs 3 integers [s1,s2,s3]')
            return
        self.last_thetas = self.model.steps_to_rad(tuple(data[:3]))

    def on_timer(self):
        try:
            self.viz.draw_state(self.last_thetas)
        except Exception as e:
            self.get_logger().error(f'viz error: {e}')

def main():
    rclpy.init()
    node = DeltaVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    plt.close('all')

if __name__ == '__main__':
    main()
