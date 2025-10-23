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

    MotorCmd layout: [motor_id, vel, direction, off_us]
      - motor_id: 0=A, 1=B, 2=C
      - vel: magnitude (usually 0 or 1)
      - direction: -1, 0, +1  (sign)
      - off_us: integer (step LOW time)

    Output to Arduino (choose with param 'format'):
      - 'kv'  (default): "motor=A vel=1 off_us=800\n"
      - 'csv':           "0,1,800\n"
    """
    def __init__(self):
        super().__init__('serial_bridge')

        # ----- Parameters -----
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 50.0)
        self.declare_parameter('format', 'kv')   # 'kv' or 'csv'

        port   = self.get_parameter('port').value
        baud   = int(self.get_parameter('baud').value)
        rate   = float(self.get_parameter('rate_hz').value)
        fmt    = str(self.get_parameter('format').value).lower()

        if fmt not in ('kv', 'csv'):
            self.get_logger().warn("param 'format' must be 'kv' or 'csv'; falling back to 'kv'")
            fmt = 'kv'
        self.format = fmt

        # ----- Serial -----
        self.ser = serial.Serial(port, baud, timeout=0.02)
        time.sleep(2.0)  # allow Uno reset
        self.get_logger().info(f"Opened {port} @ {baud} (format={self.format})")

        # ----- Latest command -----
        self.last_cmd = None  # list/tuple: [motor_id, vel, direction, off_us]

        # ----- ROS sub & timer -----
        self.sub = self.create_subscription(MotorCmd, 'motor_cmd', self.on_cmd, 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

    # ---- Callbacks ----
    def on_cmd(self, msg: MotorCmd):
        # Expect 4 elements
        if len(msg.data) < 4:
            self.get_logger().warn(f"motor_cmd has {len(msg.data)} elems; need 4. Ignoring.")
            return
        self.last_cmd = list(msg.data)

    def tick(self):
        if self.last_cmd is None:
            return

        try:
            motor_id = int(round(self.last_cmd[0]))
            vel_mag  = int(round(self.last_cmd[1]))      # expected 0 or 1
            direction = int(round(self.last_cmd[2]))     # -1, 0, +1
            off_us   = int(round(self.last_cmd[3]))

            # Combine magnitude + direction into signed velocity for Arduino
            signed_vel = vel_mag * direction
            if signed_vel < -1: signed_vel = -1
            if signed_vel >  1: signed_vel =  1

            # Build line for Arduino
            if self.format == 'csv':
                # e.g. "0,1,800\n"
                line = f"{motor_id},{signed_vel},{off_us}\n"
            else:
                # kv format e.g. "motor=A vel=1 off_us=800\n"
                motor_letter = LETTER.get(motor_id, 'A')
                line = f"motor={motor_letter} vel={signed_vel} off_us={off_us}\n"

            # Send
            self.ser.write(line.encode('ascii'))
            # Optional debug (comment out if too chatty)
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
