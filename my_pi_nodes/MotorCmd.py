# my_pi_nodes/delta_control.py
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray as MotorCmd
from geometry_msgs.msg import Point   # <-- NEW

from .Control import dynamics  # your dynamics class, including FK/Jacobian

# --- hardware / timing constants ---
DEG_PER_STEP = 0.045        # deg per motor step (given)
ON_US        = 5.0         # high time for step pulse
OFF_US_MIN   = 2000.0    # fastest (smallest delay)
OFF_US_MAX   = 2800.0      # slowest (largest delay)

# max physical step rate (steps/s) at OFF_US_MIN
MAX_STEP_FREQ = 1e6 / (ON_US + OFF_US_MIN)     # ~ 499 steps/s
# corresponding max joint speed (deg/s)
MAX_QDOT   =.4 *  MAX_STEP_FREQ * DEG_PER_STEP   # ~ 45 deg/s

class DeltaControl(Node):
    def __init__(self):
        super().__init__('delta_control')

        # --- state ---
        thetas0 = np.array([0.0, 0.0, 0.0])
        self.ctrl = dynamics(thetas0)
        self.last_joy = Joy()

        # --- ROS I/O ---
        self.sub_joy = self.create_subscription(
            Joy, 'joy', self.on_joy, 10
        )
        self.pub_motor = self.create_publisher(
            MotorCmd, 'motor_cmd', 10
        )

        # NEW: publisher for tip position
        self.pub_tip = self.create_publisher(
            Point, 'tip_position', 10
        )

        # control loop at 50 Hz
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.on_timer)

    def on_joy(self, msg: Joy):
        self.last_joy = msg

    def joy_to_tip_vel(self) -> np.ndarray:
        axes = self.last_joy.axes if self.last_joy.axes else [0.0]*6

        # Example mapping:
        # Left stick X -> x velocity, left stick Y -> y velocity, right stick Y -> z
        ax_x = axes[0]    # [-1..1]
        ax_y = axes[1]
        ax_z = axes[4] if len(axes) > 4 else 0.0

        vx =  ax_x
        vy = -ax_y
        vz = -ax_z

        return np.array([vx, vy, vz], dtype=float)

    def on_timer(self):
        # 1) Desired tip velocity from joystick
        v = self.joy_to_tip_vel()   # “joystick units”
        v[np.abs(v) < 0.1] = 0.0

        # If joystick centered, send "stop" to all motors
        if np.linalg.norm(v) < 1e-3:
            msg = MotorCmd()
            msg.data = [0.0, 0.0, 0.0]
            self.pub_motor.publish(msg)

            # Still useful to publish current tip position (not moving)
            position = self.ctrl.fk(self.ctrl.thetas)
            self.publish_tip_position(position)
            return
        
        # 2) Compute qdot via your Jacobian-based method (deg/s)
        qdot = self.ctrl.qdot_from_v(v)
        print(f"Following is Qdot {qdot}")
        # 3) Limit qdot to physical range
        max_abs = np.max(np.abs(qdot))
        if max_abs > MAX_QDOT:
            qdot = qdot * (MAX_QDOT / max_abs)

        # 4) Integrate thetas so our model keeps up (open-loop)
        self.ctrl.thetas = self.ctrl.thetas + qdot * self.dt
        position = self.ctrl.fk(self.ctrl.thetas)
        print(f"Thetas: {self.ctrl.thetas}, position: {position}")

        # --- NEW: publish tip position ---
        self.publish_tip_position(position)

        # 5) Map qdot -> signed off_us for each motor
        scaled = []
        for qd in qdot:
            if abs(qd) < 1e-3:
                # treat very small speeds as "stopped"
                scaled.append(0.0)
                continue

            # normalize magnitude to [0, 1]
            s = min(abs(qd) / MAX_QDOT, 1.0)

            # map s in [0,1] -> off_us in [OFF_US_MAX, OFF_US_MIN]
            # s=0 -> OFF_US_MAX (slow), s=1 -> OFF_US_MIN (fast)
            off_us = OFF_US_MAX - s * (OFF_US_MAX - OFF_US_MIN)

            # encode direction in the sign
            val = math.copysign(off_us, qd)
            scaled.append(val)

        # 6) Build MotorCmd: [valA, valB, valC]
        m = MotorCmd()
        m.data = [float(scaled[0]), float(scaled[1]), float(scaled[2])]
        self.pub_motor.publish(m)

    def publish_tip_position(self, position: np.ndarray):
        """Publish tip position as geometry_msgs/Point."""
        p = Point()
        # assuming fk returns [x, y, z]
        p.x = float(position[0])
        p.y = float(position[1])
        p.z = float(position[2])
        self.pub_tip.publish(p)

def main():
    rclpy.init()
    node = DeltaControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
