# my_pi_nodes/my_pi_nodes/tip_plotter.py

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import matplotlib
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  needed for 3D

class TipPlotter(Node):
    def __init__(self):
        super().__init__('tip_plotter')

        # ---- ROS subscriber ----
        self.sub = self.create_subscription(
            Float32MultiArray,
            'tip_position',      # <-- change if your topic is different
            self.on_tip,
            10
        )

        # latest tip position (x, y, z)
        self.latest = None

        # ---- Matplotlib setup (main thread) ----
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')

        # set some rough workspace limits (tune these)
        self.ax.set_xlim(-300, 300)
        self.ax.set_ylim(-300, 300)
        self.ax.set_zlim(-600, 0)

        # a single point (we’ll update its position)
        self.scatter = self.ax.scatter([0], [0], [0])

        # timer to refresh plot
        self.timer = self.create_timer(0.05, self.update_plot)  # 20 Hz

        self.get_logger().info("TipPlotter node started.")

    def on_tip(self, msg: Float32MultiArray):
        if len(msg.data) < 3:
            self.get_logger().warn(f"tip_position msg too short: {msg.data}")
            return
        x, y, z = msg.data[0], msg.data[1], msg.data[2]
        self.latest = (x, y, z)

    def update_plot(self):
        if self.latest is None:
            return

        x, y, z = self.latest

        # update the single point
        self.scatter._offsets3d = ([x], [y], [z])

        # optional: adjust axes if needed
        # self.ax.set_xlim(x - 200, x + 200)
        # self.ax.set_ylim(y - 200, y + 200)
        # self.ax.set_zlim(z - 200, z + 200)

        self.fig.canvas.draw_idle()
        # let GUI process events
        plt.pause(0.001)


def main(args=None):
    rclpy.init(args=args)

    node = TipPlotter()

    # turn on interactive mode and show window without blocking
    plt.ion()
    plt.show(block=False)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
    # keep window open a bit after shutdown if you like
    # plt.ioff()
    # plt.show()


if __name__ == '__main__':
    main()
