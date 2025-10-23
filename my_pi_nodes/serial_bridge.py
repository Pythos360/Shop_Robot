# my_pi_nodes/serial_bridge.py
import time
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray as MotorCmd

LETTER = {0: 'A', 1: 'B', 2: 'C'}

class SerialBridge(Node):
    """
    Bridges MotorCmd (Float32MultiArray) -> Arduino over serial.

    Accepts either:
      len=3: [motor_id, signed_vel, off_us]
      len=4: [motor_id, vel_mag, direction, off_us]

    Output to Arduino (param 'format'):
      'kv'  (default): "motor=A vel=1 off_us=800\n"
      'csv'           : "0,1,800\n"
    """
    def __init__(self):
        super().__init__('serial_bridge')

        # Parameters
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
        time.sleep(2.0)  # allow Uno reset
        self.get_logger().info(f"Opened {port} @ {baud} (format={self.format})")

        # Latest cmd
        self.last_cmd = None

        # ROS I/O
        self.sub = self.create_subscription(MotorCmd, 'motor_cmd', self.on_cmd, 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def on_cmd(self, msg: MotorCmd):
        data = list(msg.data)
        if len(data) not in (3, 4):
            self.get_logger().warn(f"motor_cmd has {len(data)} elems; need 3 or 4. Ignoring.")
            return
        self.last_cmd = data

    def tick(self):
        if self.last_cmd is None:
            return

        try:
            data = self.last_cmd

            if len(data) == 4:
                # [motor_id, vel_mag, direction, off_us]
                motor_id  = int(round(data[0]))
                vel_mag   = int(round(data[1]))        # 0 or 1
                direction = int(round(data[2]))        # -1,0,+1
                off_us    = int(round(data[3]))
                signed_vel = max(-1, min(1, vel_mag * direction))
            else:
                # len==3 -> [motor_id, signed_vel, off_us]
                motor_id   = int(round(data[0]))
                signed_vel = int(round(data[1]))       # -1,0,+1
                off_us     = int(round(data[2]))
                # normalize just in case
                if signed_vel < -1: signed_vel = -1
                if signed_vel >  1: signed_vel =  1

            # Build line for Arduino
            if self.format == 'csv':
                # "0,1,800"
                line = f"{motor_id},{signed_vel},{off_us}\n"
            else:
                # "motor=A vel=1 off_us=800"
                motor_letter = LETTER.get(motor_id, 'A')
                line = f"motor={motor_letter} vel={signed_vel} off_us={off_us}\n"

            self.ser.write(line.encode('ascii'))
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
