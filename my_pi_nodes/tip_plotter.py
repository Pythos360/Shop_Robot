# my_pi_nodes/tip_plotter.py
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 (needed for 3D)
from matplotlib.animation import FuncAnimation


class TipPlotter(Node):
    def __init__(self):
        super().__init__('tip_plotter')

        # --- ROS sub ---
        self.sub = self.create_subscription(
            Float32MultiArray,
            'tip_position',
            self.on_tip,
            10
        )

        # last known position of the tip [x, y, z]
        self.pos = np.zeros(3)

        # --- Matplotlib setup ---
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection='3d')

        # nice-ish bounds for your delta workspace (tune as needed)
        self.ax.set_xlim(-500, 500)
        self.ax.set_ylim(-500, 500)
        self.ax.set_zlim(-600, -200)
        self.ax.set_xlabel('X (mm)')
        self.ax.set_ylabel('Y (mm)')
        self.ax.set_zlabel('Z (mm)')
        self.ax.set_title('Delta tip position')

        # a single point we will keep updating
        (self.point,) = self.ax.plot([], [], [], 'o')

        # IMPORTANT: keep a reference to the animation so it isn't GC'd
        self.ani = FuncAnimation(
            self.fig,
            self.update_plot,
            interval=50,   # ms
            blit=False
        )

    def on_tip(self, msg: Float32MultiArray):
        # expect [x, y, z] in msg.data
        if len(msg.data) >= 3:
            self.pos[0] = msg.data[0]
            self.pos[1] = msg.data[1]
            self.pos[2] = msg.data[2]

    def update_plot(self, frame):
        # update the dot to the latest position
        x, y, z = self.pos
        self.point.set_data([x], [y])
        self.point.set_3d_properties([z])
        return (self.point,)


def main():
    rclpy.init()
    node = TipPlotter()

    # run plt.show() in a separate thread so rclpy.spin() can still run
    t = threading.Thread(target=plt.show, daemon=True)
    t.start()

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
