# MotorCmd_Test.py
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray as MotorCmd
from geometry_msgs.msg import Point

from .Control_Test import dynamics, qdot_to_off_us_physical


# -----------------------------
# Stepper + scheduler parameters
# -----------------------------
DEG_PER_STEP = 0.045
ON_US = 5.0

# IMPORTANT:
# Large OFF_US_MAX is required if you want slow motion.
# Example: 1 deg/s => off_us ~ 45,000 us (with DEG_PER_STEP=0.045)
OFF_US_MIN = 1000.0
OFF_US_MAX = 100000.0

# Treat tiny qdot as STOP to avoid noise
QDOT_STOP = 0.05  # deg/s (tune 0.05–0.2)

# -----------------------------
# Control loop parameters
# -----------------------------
DT = 0.02  # 50 Hz
JOY_DEADZONE = 0.10
JOY_GAIN = 1.0
CART_GAIN = 5.0  # scales joystick command inside qdot_from_v


class MotorCmdTest(Node):
    def __init__(self):
        super().__init__("motorcmd_test")

        self.ctrl = dynamics([0.0, 0.0, 0.0])
        self.last_joy = Joy()

        self.sub_joy = self.create_subscription(Joy, "joy", self.on_joy, 10)
        self.pub_motor = self.create_publisher(MotorCmd, "motor_cmd", 10)
        self.pub_tip = self.create_publisher(Point, "tip_position", 10)

        self.timer = self.create_timer(DT, self.on_timer)

    def on_joy(self, msg: Joy):
        self.last_joy = msg

    def joy_to_tip_vel(self) -> np.ndarray:
        axes = self.last_joy.axes if self.last_joy.axes else []
        ax_x = axes[0] if len(axes) > 0 else 0.0
        ax_y = axes[1] if len(axes) > 1 else 0.0
        ax_z = axes[4] if len(axes) > 4 else 0.0

        v = JOY_GAIN * np.array([ax_x, -ax_y, -ax_z], dtype=float)
        v[np.abs(v) < JOY_DEADZONE] = 0.0
        return v

    def publish_tip(self, xyz: np.ndarray):
        p = Point()
        p.x = float(xyz[0])
        p.y = float(xyz[1])
        p.z = float(xyz[2])
        self.pub_tip.publish(p)

    def on_timer(self):
        v = self.joy_to_tip_vel()

        if np.linalg.norm(v) < 1e-9:
            qdot = np.zeros(3, dtype=float)
            # no integration when stopped
        else:
            _, qdot = self.ctrl.step(DT, v=v, gain=CART_GAIN, deadzone=0.0)

        # Tip position publish
        tip = self.ctrl.fk(self.ctrl.thetas)
        self.publish_tip(tip)

        # Convert qdot -> signed off_us (physical mapping)
        off_us = qdot_to_off_us_physical(
            qdot,
            deg_per_step=DEG_PER_STEP,
            on_us=ON_US,
            off_us_min=OFF_US_MIN,
            off_us_max=OFF_US_MAX,
            qdot_stop=QDOT_STOP,
        )

        m = MotorCmd()
        m.data = [float(off_us[0]), float(off_us[1]), float(off_us[2])]
        self.pub_motor.publish(m)


def main(args=None):
    rclpy.init(args=args)
    node = MotorCmdTest()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
