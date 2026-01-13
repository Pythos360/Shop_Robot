# my_pi_nodes/delta_control_clean.py
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray as MotorCmd
from geometry_msgs.msg import Point

from .Control_Test import dynamics


# -----------------------------
# Hardware / timing constants
# -----------------------------
DEG_PER_STEP = 0.045
ON_US = 5.0
OFF_US_MIN = 1000.0
OFF_US_MAX = 2800.0

MAX_STEP_FREQ = 1e6 / (ON_US + OFF_US_MIN)                 # steps/s at fastest possible
MAX_QDOT = 0.4 * MAX_STEP_FREQ * DEG_PER_STEP              # deg/s safety cap


class DeltaControl(Node):
    def __init__(self):
        super().__init__("delta_control")

        # ---- control core ----
        thetas0 = np.array([0.0, 0.0, 0.0], dtype=float)
        self.ctrl = dynamics(thetas0)

        # ---- joystick settings ----
        self.last_joy = Joy()
        self.joy_deadzone = 0.10
        self.joy_gain = 1.0          # scales joystick vector before passing to ctrl.step
        self.cart_gain = 5.0         # passed into ctrl.step -> qdot_from_v scaling


        # ---- ROS I/O ----
        self.sub_joy = self.create_subscription(Joy, "joy", self.on_joy, 10)
        self.sub_theta_target = self.create_subscription(MotorCmd, "theta_target", self.on_theta_target, 10)
        self.sub_target = self.create_subscription(Point, "move_target", self.on_target, 10)

        self.pub_motor = self.create_publisher(MotorCmd, "motor_cmd", 10)
        self.pub_tip = self.create_publisher(Point, "tip_position", 10)

        # ---- loop ----
        self.dt = 0.02  # 50 Hz
        self.timer = self.create_timer(self.dt, self.on_timer)

    # -----------------------------
    # ROS callbacks
    # -----------------------------
    def on_target(self, msg: Point):
        target = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.get_logger().info(f"New move_target: {target}")
        self.mover.set_target(target)

    def on_theta_target(self, msg: MotorCmd):
        data = list(msg.data)
        if len(data) != 3:
            self.get_logger().warn(f"theta_target needs 3 elements, got {len(data)}")
            return
        target = np.array(data, dtype=float)
        self.get_logger().info(f"New theta_target: {target}")
        self.theta_mover.set_target(target)

    def on_joy(self, msg: Joy):
        self.last_joy = msg

        # button[1] cancels Cartesian move (keep your existing behavior)
        if len(msg.buttons) > 1 and msg.buttons[1]:
            self.get_logger().info("Move cancelled by joystick button.")
            self.mover.stop()

    # -----------------------------
    # Helpers
    # -----------------------------
    def joy_to_tip_vel(self) -> np.ndarray:
        """
        Map joystick axes -> tip velocity command vector [vx, vy, vz].
        This is a *command* vector; ctrl.step() will apply `cart_gain`.
        """
        axes = self.last_joy.axes if self.last_joy.axes else []

        ax_x = axes[0] if len(axes) > 0 else 0.0
        ax_y = axes[1] if len(axes) > 1 else 0.0
        ax_z = axes[4] if len(axes) > 4 else 0.0

        vx =  ax_x
        vy = -ax_y
        vz = -ax_z

        v = self.joy_gain * np.array([vx, vy, vz], dtype=float)
        v[np.abs(v) < self.joy_deadzone] = 0.0
        return v

    def limit_qdot(self, qdot: np.ndarray) -> np.ndarray:
        qdot = np.asarray(qdot, dtype=float)
        max_abs = float(np.max(np.abs(qdot))) if qdot.size else 0.0
        if max_abs > MAX_QDOT and max_abs > 1e-9:
            qdot = qdot * (MAX_QDOT / max_abs)
        return qdot

    def qdot_to_off_us(self, qdot: np.ndarray) -> np.ndarray:
        """
        Convert qdot (deg/s) -> signed off_us.
        """
        qdot = np.asarray(qdot, dtype=float)
        out = np.zeros(3, dtype=float)

        for i, qd in enumerate(qdot):
            if abs(qd) < 1e-6:
                out[i] = 0.0
                continue

            s = min(abs(qd) / MAX_QDOT, 1.0)  # 0..1
            off_us = OFF_US_MAX - s * (OFF_US_MAX - OFF_US_MIN)
            out[i] = math.copysign(off_us, qd)

        return out

    def publish_tip(self, xyz: np.ndarray):
        p = Point()
        p.x = float(xyz[0])
        p.y = float(xyz[1])
        p.z = float(xyz[2])
        self.pub_tip.publish(p)

    # -----------------------------
    # Main loop
    # -----------------------------
    def on_timer(self):
        dt = self.dt

        # Priority: theta target > Cartesian target > joystick
        if self.theta_mover.active:
            qdot = self.theta_mover.update(dt)
            qdot = self.limit_qdot(qdot)
            self.ctrl.thetas = self.ctrl.thetas + qdot * dt

        elif self.mover.active:
            qdot = self.mover.update(dt)
            qdot = self.limit_qdot(qdot)
            self.ctrl.thetas = self.ctrl.thetas + qdot * dt

        else:
            # Joystick mode uses the control core to integrate
            v = self.joy_to_tip_vel()

            if np.linalg.norm(v) < 1e-6:
                qdot = np.zeros(3, dtype=float)
                # no integration
            else:
                # Disable deadzone inside ctrl since we already applied it above
                thetas, qdot = self.ctrl.step(
                    dt,
                    v=v,
                    gain=self.cart_gain,
                    deadzone=0.0
                )

                # Enforce MAX_QDOT (and *re-integrate* safely if needed)
                qdot_limited = self.limit_qdot(qdot)
                if np.any(np.abs(qdot_limited - qdot) > 1e-12):
                    # redo integration with limited qdot to avoid overshoot
                    self.ctrl.thetas = self.ctrl.thetas - qdot * dt + qdot_limited * dt
                    qdot = qdot_limited

        # Tip position publish
        tip = self.ctrl.fk(self.ctrl.thetas)
        self.publish_tip(tip)

        # Motor command publish
        off_us = self.qdot_to_off_us(qdot)
        m = MotorCmd()
        m.data = [float(off_us[0]), float(off_us[1]), float(off_us[2])]
        self.pub_motor.publish(m)


def main():
    rclpy.init()
    node = DeltaControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


def main():
    rclpy.init()
    node = DeltaControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

