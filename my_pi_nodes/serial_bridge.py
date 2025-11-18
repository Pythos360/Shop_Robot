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
        # For the new Arduino code we always use CSV: "cmdA,cmdB,cmdC\n"
        self.declare_parameter('format', 'csv')

        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        rate = float(self.get_parameter('rate_hz').value)
        fmt = str(self.get_parameter('format').value).lower()
        if fmt != 'csv':
            self.get_logger().warn(
                "param 'format' must be 'csv' for the new Arduino code; forcing 'csv'"
            )
            fmt = 'csv'
        self.format = fmt

        # Serial (non-blocking read)
        self.ser = serial.Serial(port, baud, timeout=0.0)
        time.sleep(2.0)
        self.get_logger().info(f"Opened {port} @ {baud} (format={self.format})")

        # We now expect exactly 3 elements: [cmdA, cmdB, cmdC]
        self.last_cmd = None

        # Buffer for incoming Arduino text
        self._rx_buf = ""

        # ROS I/O
        self.sub = self.create_subscription(
            MotorCmd,
            'motor_cmd',
            self.on_cmd,
            50
        )
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def on_cmd(self, msg: MotorCmd):
        data = list(msg.data)
        if len(data) != 3:
            self.get_logger().warn(
                f"motor_cmd has {len(data)} elems; need exactly 3: [cmdA, cmdB, cmdC]. Skipping."
            )
            return
        self.last_cmd = data  # atomic replace

    def _read_arduino(self):
        """Non-blocking read; log any full lines from the Arduino."""
        try:
            n = self.ser.in_waiting
            if not n:
                return
            chunk = self.ser.read(n)
            if not chunk:
                return

            try:
                text = chunk.decode('utf-8', errors='ignore')
            except Exception:
                return

            self._rx_buf += text
            while '\n' in self._rx_buf:
                line, self._rx_buf = self._rx_buf.split('\n', 1)
                line = line.strip()
                if line:
                    # Prefix so you can distinguish Arduino prints
                    self.get_logger().info(f"[ARD] {line}")

        except Exception as e:
            self.get_logger().warn(f"Serial read error: {e}")

    def tick(self):
        # 1) Read any prints from Arduino
        self._read_arduino()

        # 2) Send latest motor command
        data = self.last_cmd
        if not data or len(data) != 3:
            return

        try:
            # Convert to ints so Arduino can parse with "%ld,%ld,%ld"
            a = int(round(float(data[0])))
            b = int(round(float(data[1])))
            c = int(round(float(data[2])))

            line = f"{a},{b},{c}\n"
            self.ser.write(line.encode('ascii'))

            # Change to debug if too chatty
            self.get_logger().debug(f"Sent: {line.strip()}")

        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
