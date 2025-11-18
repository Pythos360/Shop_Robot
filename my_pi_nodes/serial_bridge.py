# my_pi_nodes/serial_bridge.py
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as MotorCmd

def fmt_val(x):  # compact floats
    try:
        return f"{float(x):.6g}"
    except Exception:
        return "0"

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 50.0)
        # We always use CSV now; Arduino expects "cmdA,cmdB,cmdC"
        self.declare_parameter('format', 'csv')

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        rate = float(self.get_parameter('rate_hz').value)
        fmt  = str(self.get_parameter('format').value).lower()
        if fmt != 'csv':
            self.get_logger().warn("param 'format' must be 'csv' for the new Arduino code; forcing 'csv'")
            fmt = 'csv'
        self.format = fmt  # kept for completeness

        # Serial
        self.ser = serial.Serial(port, baud, timeout=0.02)
        time.sleep(2.0)
        self.get_logger().info(f"Opened {port} @ {baud} (format={self.format})")

        # Now we expect exactly 3 elements: [cmdA, cmdB, cmdC]
        self.last_cmd = None

        # ROS I/O
        self.sub = self.create_subscription(MotorCmd, 'motor_cmd', self.on_cmd, 50)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def on_cmd(self, msg: MotorCmd):
        data = list(msg.data)
        if len(data) != 3:
            self.get_logger().warn(
                f"motor_cmd has {len(data)} elems; need exactly 3: [cmdA, cmdB, cmdC]. Skipping."
            )
            return
        self.last_cmd = data  # atomic replace

    def tick(self):
        data = self.last_cmd
        if not data or len(data) != 3:
            return

        try:
            # Arduino expects: "cmdA,cmdB,cmdC\n"
            line = f"{fmt_val(data[0])},{fmt_val(data[1])},{fmt_val(data[2])}\n"
            self.ser.write(line.encode('ascii'))

            # You can change this to .debug if the spam is too much
            self.get_logger().info(f"Sent: {line.strip()}")

        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

def main():
    rclpy.init()
    node = SerialBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
