import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray as MotorCmd

BTN_SPEED_DOWN = 5  # slower
BTN_SPEED_UP   = 4  # faster

AXIS_A = 0
AXIS_B = 1
AXIS_C = 3

MIN_OFF_US = 100
MAX_OFF_US = 5000
DEADBAND   = 0.05

def vel_from_axis(x: float) -> int:
    if x >  DEADBAND: return +1
    if x < -DEADBAND: return -1
    return 0

class JoyToStep(Node):
    def __init__(self):
        super().__init__('joy_to_step')
        self.pub = self.create_publisher(MotorCmd, 'motor_cmd', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.on_joy, 10)
        self.off_us = 2800
        self.prev_buttons = []  # needed for edge detection

    def _btn(self, buttons, i):
        return (i < len(buttons)) and (buttons[i] == 1)

    def _axis(self, axes, i):
        return axes[i] if i < len(axes) else 0.0

    def _rose(self, buttons, i):
        prev = self.prev_buttons[i] if i < len(self.prev_buttons) else 0
        now  = 1 if (i < len(buttons) and buttons[i] == 1) else 0
        return prev == 0 and now == 1

    def _publish_cmd(self, motor_id: int, vel: int):
        msg = MotorCmd()
        # len=3 format expected by serial_bridge: [motor_id, signed_vel, off_us]
        msg.data = [float(motor_id), float(vel), float(self.off_us)]
        self.pub.publish(msg)

    def on_joy(self, msg: Joy):
        # velocities from three axes
        velA = vel_from_axis(self._axis(msg.axes, AXIS_A))
        velB = vel_from_axis(self._axis(msg.axes, AXIS_B))
        velC = vel_from_axis(self._axis(msg.axes, AXIS_C))

        # speed up/down on rising edges
        if self._rose(msg.buttons, BTN_SPEED_UP):
            self.off_us = max(MIN_OFF_US, self.off_us - 100)
        if self._rose(msg.buttons, BTN_SPEED_DOWN):
            self.off_us = min(MAX_OFF_US, self.off_us + 100)

        # publish three commands (one per motor)
        self._publish_cmd(0, velA)  # A
        self._publish_cmd(1, velB)  # B
        self._publish_cmd(2, velC)  # C

        # log & update edge state
        self.get_logger().info(f"velA={velA} velB={velB} velC={velC} off_us={self.off_us}")
        self.prev_buttons = list(msg.buttons)

def main():
    rclpy.init()
    node = JoyToStep()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
