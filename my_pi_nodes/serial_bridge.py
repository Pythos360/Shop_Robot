
# my_pi_nodes/serial_bridge.py
import time
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('rate_hz', 50.0)


        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        

        # Serial
        self.ser = serial.Serial(port, baud, timeout=0.02)
        time.sleep(2.0)  # allow Uno reset
        self.get_logger().info(f"Opened {port} @ {baud}")

        # Sub to joystick
        self.joy = Joy()
        self.sub = self.create_subscription(Joy, 'joy', self.on_joy, 10)

        # Send periodically
        self.timer = self.create_timer(1.0/float(self.get_parameter('rate_hz').value), self.tick)

    def on_joy(self, msg: Joy):
        self.joy = msg

    def tick(self):
        
        axes = list(getattr(self.joy, "axes", []) or [])
        buttons = list(getattr(self.joy, "buttons", []) or [])

        a_part = ",".join(f"{x:.6g}" for x in axes)
        b_part = ",".join(str(int(x)) for x in buttons)

        line = f"A:{a_part};B:{b_part}\n"
        try:
            self.ser.write(line.encode('ascii'))
            self.get_logger().info(f"Sent: {line}")
            # Optionally read an ack:
            # ack = self.ser.readline().decode(errors='ignore').strip()
            # if ack: self.get_logger().debug(ack)
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
