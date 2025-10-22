import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from motor_control.msg import MotorCmd

AXIS_THROTTLE = 1   # stick Y in [-1,1]
BTN_ENABLE    = 0   # A button index

MIN_OFF_US = 100
MAX_OFF_US = 5000

class JoyToStep(Node):
    def __init__(self):
        super().__init__('joy_to_step')
        self.pub = self.create_publisher(MotorCmd, 'motor_cmd', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.on_joy, 10)

    def on_joy(self, msg: Joy):
        enable = (msg.buttons[BTN_ENABLE] == 1)
        v = msg.axes[AXIS_THROTTLE] if len(msg.axes) > AXIS_THROTTLE else 0.0
        # sign = direction, magnitude -> off_us (smaller = faster)
        direction = 0
        if abs(v) > 0.05:            # deadband
            direction = 1 if v > 0 else -1
        mag = min(1.0, max(0.0, abs(v)))
        off_us = int(MAX_OFF_US - mag * (MAX_OFF_US - MIN_OFF_US))
        cmd = MotorCmd(enable=enable, direction=direction, step_us=off_us)
        self.pub.publish(cmd)

def main():
    rclpy.init()
    rclpy.spin(JoyToStep())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
