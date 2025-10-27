# my_pi_nodes/serial_bridge.py
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as MotorCmd

def fmt_val(x):
    return f"{x:.6g}"

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('format', 'kv')   # 'kv' or 'csv'

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        rate = float(self.get_parameter('rate_hz').value)
        fmt  = str(self.get_parameter('format').value).lower()
        if fmt not in ('kv', 'csv'):
            self.get_logger().warn("param 'format' must be 'kv' or 'csv'; falling back to 'kv'")
            fmt = 'kv'
        self.format = fmt

        # Serial
        self.ser = serial.Serial(port, baud, timeout=0.02)
        time.sleep(2.0)
        self.get_logger().info(f"Opened {port} @ {baud} (format={self.format})")

        # Latest packet (expected len==4)
        self.last_cmd = None

        # ROS
        self.sub = self.create_subscription(MotorCmd, 'motor_cmd', self.on_cmd, 50)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def on_cmd(self, msg: MotorCmd):
        data = list(msg.data)
        if len(data) != 4:
            self.get_logger().warn(
                f"motor_cmd has {len(data)} elems; need exactly 4: [velA, velB, velC, off_us]. Skipping.")
            return
        self.last_cmd = data  # atomic replacement

    def tick(self):
        # Snapshot to avoid races with on_cmd
        data = self.last_cmd
        if not data or len(data) != 4:
            return  # nothing valid to send yet

        try:
            velA, velB, velC, off_us = data
            if self.format == 'csv':
                line = f"{fmt_val(velA)},{fmt_val(velB)},{fmt_val(velC)},{fmt_val(off_us)}\n"
            else:
                line = (f"velA={fmt_val(velA)} "
                        f"velB={fmt_val(velB)} "
                        f"velC={fmt_val(velC)} "
                        f"off_us={fmt_val(off_us)}\n")

            self.ser.write(line.encode('ascii'))
            self.get_logger().info(f"Sent: {line.strip()}")

        except Exception as e:
            # Do NOT reference velA/velB/velC/off_us here; just log the exception.
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
