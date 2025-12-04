# my_pi_nodes/delta_control.py
import math
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray as MotorCmd
from geometry_msgs.msg import Point   

from .Control import dynamics, ctrl_mov, theta_mov  

# --- hardware / timing constants ---
DEG_PER_STEP = 0.045        
ON_US        = 5.0       
OFF_US_MIN   = 2000.0   
OFF_US_MAX   = 2800.0      


MAX_STEP_FREQ = 1e6 / (ON_US + OFF_US_MIN)   

MAX_QDOT   =.4 *  MAX_STEP_FREQ * DEG_PER_STEP  

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

        self.pub_tip = self.create_publisher(
            Point, 'tip_position', 10
        )

        # control loop at 50 Hz
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.on_timer)
        # --- motion controller: position targets ---
        self.mover = ctrl_mov(
            self.ctrl,
            kp=0.8,           
            max_tip_speed=30., 
            tol=1.0          
        )

        self.theta_mover = theta_mov(
            self.ctrl,
            kp=2.0,
            max_qdot=MAX_QDOT,   
            tol_deg=0.5
        )

        # subscribe to joint theta targets 
        self.sub_theta_target = self.create_subscription(
            MotorCmd,
            'theta_target',
            self.on_theta_target,
            10
        )

        # subscribe to Cartesian move targets
        self.sub_target = self.create_subscription(
            Point,
            'move_target',     
            self.on_target,
            10
        )

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

        if len(msg.buttons) > 1 and msg.buttons[1]:
            self.get_logger().info("Move cancelled by joystick button.")
            self.mover.stop()

    def joy_to_tip_vel(self) -> np.ndarray:
        axes = self.last_joy.axes if self.last_joy.axes else [0.0]*6

       
        ax_x = axes[0]    
        ax_y = axes[1]
        ax_z = axes[4] if len(axes) > 4 else 0.0

        vx =  ax_x
        vy = -ax_y
        vz = -ax_z

        return np.array([vx, vy, vz], dtype=float)

    def on_timer(self):
        if self.theta_mover.active:
            # Joint-space control
            qdot = self.theta_mover.update(self.dt)

        elif self.mover.active:
            # Cartesian target control
            qdot = self.mover.update(self.dt)

        else:
            # Joystick-velocity mode
            v = self.joy_to_tip_vel()
            v[np.abs(v) < 0.1] = 0.0

            if np.linalg.norm(v) < 1e-3:
                qdot = np.zeros(3, dtype=float)
            else:
                qdot = self.ctrl.qdot_from_v(v)

        # 1) Limit qdot to physical range (MAX_QDOT)
        max_abs = np.max(np.abs(qdot))
        if max_abs > MAX_QDOT and max_abs > 1e-6:
            qdot = qdot * (MAX_QDOT / max_abs)

        # 2) Integrate thetas
        self.ctrl.thetas = self.ctrl.thetas + qdot * self.dt
        position = self.ctrl.fk(self.ctrl.thetas)
        print(f"Thetas: {self.ctrl.thetas}, position: {position}, qdot: {qdot}")

        # 3) Publish tip position
        self.publish_tip_position(position)

        # 4) Map qdot 
        scaled = []
        for qd in qdot:
            if abs(qd) < 1e-3:
                scaled.append(0.0)
                continue

            s = min(abs(qd) / MAX_QDOT, 1.0)
            off_us = OFF_US_MAX - s * (OFF_US_MAX - OFF_US_MIN)
            val = math.copysign(off_us, qd)
            scaled.append(val)

        m = MotorCmd()
        m.data = [float(scaled[0]), float(scaled[1]), float(scaled[2])]
        self.pub_motor.publish(m)



    def publish_tip_position(self, position: np.ndarray):
        """Publish tip position as geometry_msgs/Point."""
        p = Point()
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
