# my_pi_nodes/tip_plotter.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D  # needed for 3D projection

class TipPlotter(Node):
    def __init__(self):
        super().__init__('tip_plotter')

        self.sub = self.create_subscription(
            Point, 'tip_position', self.on_tip, 10
        )

        # start at origin
        self.last_pos = np.zeros(3)

    def on_tip(self, msg: Point):
        self.last_pos[:] = [msg.x, msg.y, msg.z]


def main():
    rclpy.init()
    node = TipPlotter()

    # --- Matplotlib setup ---
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # workspace limits – adjust to your robot (mm)
    ax.set_xlim3d(-500, 500)
    ax.set_ylim3d(-500, 500)
    ax.set_zlim3d(-600, 0)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('Delta Tip Position')

    # single point we’ll move around
    point_plot, = ax.plot([], [], [], 'o')

    def update(frame):
        # let ROS pump callbacks
        rclpy.spin_once(node, timeout_sec=0.0)

        x, y, z = node.last_pos
        point_plot.set_data([x], [y])
        point_plot.set_3d_properties([z])
        return point_plot,

    ani = FuncAnimation(fig, update, interval=50, blit=False)
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
