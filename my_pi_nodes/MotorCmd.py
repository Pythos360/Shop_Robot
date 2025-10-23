import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
# standard message transport; we alias it to MotorCmd for minimal code changes
from std_msgs.msg import Float32MultiArray as MotorCmd

# --- Buttons ---
BTN_MOTOR_A   = 0   # pick motor A
BTN_MOTOR_B   = 1   # pick motor B
BTN_MOTOR_C   = 2   # pick motor C
BTN_DIRECTION = 3   # toggle/hold direction (0/1)

BTN_SPEED_DOWN = 4  # slower
BTN_SPEED_UP   = 5  # faster

# --- Axes ---
AXIS_A = 0  # velocity source (ONLY Axis-A is used now)

MIN_OFF_US = 100
MAX_OFF_US = 5000
DEADBAND   = 0.05

def vel_from_axis(x: float) -> int:
    """Return -1, 0, or +1 from axis value with deadband."""
    if x >  DEADBAND:
        return +1
    if x < -DEADBAND:
        return -1
    return 0

class JoyToStep(Node):
    def __init__(self):
        super().__init__('joy_to_step')
        self.pub = self.create_publisher(MotorCmd, 'motor_cmd', 10)
        self.sub = self.create_subscription(Joy, 'joy', self.on_joy, 10)

        # state
        self.last_motor = 'A'                # 'A' | 'B' | 'C'
        self.prev_buttons = []               # for edge detection
        self.off_us = 800                    # current step-off (speed) setting

    def _btn(self, buttons, i):
        return (i < len(buttons)) and (buttons[i] == 1)

    def _axis(self, axes, i):
        return axes[i] if i < len(axes) else 0.0

    def _rose(self, buttons, i):
        prev = self.prev_buttons[i] if i < len(self.prev_buttons) else 0
        now  = 1 if (i < len(buttons) and buttons[i] == 1) else 0
        return prev == 0 and now == 1

    def on_joy(self, msg: Joy):
        # --- velocity ALWAYS from Axis-A ---
        vel = vel_from_axis(self._axis(msg.axes, AXIS_A))

        # --- select motor (if any button pressed); else keep last ---
        if   self._btn(msg.buttons, BTN_MOTOR_A):
            motor = 'A'
        elif self._btn(msg.buttons, BTN_MOTOR_B):
            motor = 'B'
        elif self._btn(msg.buttons, BTN_MOTOR_C):
            motor = 'C'
        else:
            motor = self.last_motor

        # update last_motor if selection held
        if self._btn(msg.buttons, BTN_MOTOR_A) or \
           self._btn(msg.buttons, BTN_MOTOR_B) or \
           self._btn(msg.buttons, BTN_MOTOR_C):
            self.last_motor = motor

        # --- speed up/down on button edges ---
        if self._rose(msg.buttons, BTN_SPEED_UP):
            self.off_us = max(MIN_OFF_US, self.off_us - 100)   # faster
        if self._rose(msg.buttons, BTN_SPEED_DOWN):
            self.off_us = min(MAX_OFF_US, self.off_us + 100)   # slower

        # --- direction button (0/1) ---
        

        # --- publish ---
        # data layout: [motor_id, vel, direction, off_us]
        # motor_id: 0=A, 1=B, 2=C
        motor_id = {'A': 0.0, 'B': 1.0, 'C': 2.0}.get(motor, 0.0)
        cmd = MotorCmd()
        cmd.data = [motor_id, float(vel), float(direction), float(self.off_us)]
        self.pub.publish(cmd)

        self.get_logger().info(
            f"motor={motor} vel={vel} off_us={self.off_us}"
        )

        # update prev buttons for edge detection
        self.prev_buttons = list(msg.buttons)

def main():
    rclpy.init()
    node = JoyToStep()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
