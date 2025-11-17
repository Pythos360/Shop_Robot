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
        self.declare_parameter('format', 'kv')  # 'kv' or 'csv'

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

        self.last_cmd = None  # expect len==4

        # ROS I/O
        self.sub = self.create_subscription(MotorCmd, 'motor_cmd', self.on_cmd, 50)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def on_cmd(self, msg: MotorCmd):
        data = list(msg.data)
        if len(data) != 4:
            self.get_logger().warn(
                f"motor_cmd has {len(data)} elems; need exactly 4: [velA, velB, velC, off_us]. Skipping."
            )
            return
        self.last_cmd = data  # atomic replace

    def tick(self):
        data = self.last_cmd
        if not data or len(data) != 4:
            return

        try:
            # Build line directly from array values; no locals like 'velA' to ever be undefined
            if self.format == 'csv':
                line = f"{fmt_val(data[0])},{fmt_val(data[1])},{fmt_val(data[2])},{fmt_val(data[3])}\n"
            else:
                line = (
                    f"velA={fmt_val(data[0])} "
                    f"velB={fmt_val(data[1])} "
                    f"velC={fmt_val(data[2])} "
                    f"off_us={fmt_val(data[3])}\n"
                )

            self.ser.write(line.encode('ascii'))
            self.get_logger().info(f"Sent: {line.strip()}")

        except Exception as e:
            # Never reference 'line' or any per-tick locals here
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
