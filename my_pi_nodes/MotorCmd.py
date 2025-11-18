# my_pi_nodes/delta_control.py
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray as MotorCmd

from .Control import dynamics  # your dynamics class, including FK/Jacobian

MAX_TIP_SPEED = 50.0   # mm/s (tune)
MAX_QDOT      = 3   # deg/s (tune)
BASE_OFF_US   = 2000.0 # starting "slow" speed (tune)

class DeltaControl(Node):
    def __init__(self):
        super().__init__('delta_control')

        # --- state ---
        # find a valid initial theta via IK for some nominal pose
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

        # control loop at 50 Hz
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.on_timer)

    def on_joy(self, msg: Joy):
        self.last_joy = msg

    def joy_to_tip_vel(self) -> np.ndarray:
        axes = self.last_joy.axes if self.last_joy.axes else [0.0]*4

        # Example mapping:
        # Left stick X -> x velocity, left stick Y -> y velocity, right stick Y -> z
        ax_x = axes[0]    # [-1..1]
        ax_y = axes[1]
        ax_z = axes[4] if len(axes) > 3 else 0.0

        vx = MAX_TIP_SPEED * ax_x
        vy = MAX_TIP_SPEED * ax_y
        vz = MAX_TIP_SPEED * ax_z

        return np.array([vx, vy, vz], dtype=float)

    def on_timer(self):
        # 1) Desired tip velocity from joystick
        v = self.joy_to_tip_vel()   # mm/s

        # If joystick centered, stop:
        if np.linalg.norm(v) < 1e-1:
            cmd = MotorCmd()
            cmd.data = [0.0, 0.0, 0.0, BASE_OFF_US]
            self.pub_motor.publish(cmd)
            return

        # 2) Compute qdot via your Jacobian-based method
        qdot = self.ctrl.qdot_from_v(v)   # you'll add this method, or reuse qdot()

        # 3) Limit qdot
        qdot = np.clip(qdot, -MAX_QDOT, MAX_QDOT)

        # 4) Normalize to [-1, 1] for each motor
        vel = qdot / MAX_QDOT

        # 5) Integrate thetas so our model keeps up (open-loop)
        self.ctrl.thetas = self.ctrl.thetas + qdot * self.dt

        # 6) Build MotorCmd for SerialBridge: [velA, velB, velC, off_us]
        m = MotorCmd()
        m.data = [float(vel[0]), float(vel[1]), float(vel[2]), float(BASE_OFF_US)]
        self.pub_motor.publish(m)

        # optional: log at low rate
        # self.get_logger().info(f"qdot={qdot}, vel={vel}")

def main():
    rclpy.init()
    node = DeltaControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
