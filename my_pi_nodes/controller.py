import os
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")

import time, pygame
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        self.pub = self.create_publisher(Joy, 'joy', 10)

        pygame.init()
        pygame.joystick.init()
        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick found")
            rclpy.shutdown()
            raise SystemExit(1)

        self.js = pygame.joystick.Joystick(0)
        self.js.init()
        self.get_logger().info(
            f"Joystick: {self.js.get_name()} | axes={self.js.get_numaxes()} "
            f"buttons={self.js.get_numbuttons()} hats={self.js.get_numhats()}"
        )

        # Optional params to avoid index confusion
        self.declare_parameter('include_hats_as_axes', True)
        self.include_hats_as_axes = self.get_parameter('include_hats_as_axes').value

        self.timer = self.create_timer(1.0/50.0, self.tick)  # 50 Hz

    def tick(self):
        pygame.event.pump()
        axes = [float(round(self.js.get_axis(i), 4)) for i in range(self.js.get_numaxes())]
        buttons = [int(self.js.get_button(i)) for i in range(self.js.get_numbuttons())]

        if self.include_hats_as_axes:
            hats = []
            for i in range(self.js.get_numhats()):
                hx, hy = self.js.get_hat(i)
                hats.extend([float(hx), float(hy)])
            axes = axes + hats  # append at the end (document this!)

        msg = Joy()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.axes = axes
        msg.buttons = buttons
        self.pub.publish(msg)

        # DEBUG (remove later): print which axis moved most
        if axes:
            max_i = max(range(len(axes)), key=lambda i: abs(axes[i]))
            self.get_logger().debug(f"axes={axes} buttons={buttons} peak_axis={max_i}:{axes[max_i]:.3f}")

def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
