import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


BTN_MOTOR_A   = 0   # pick motor A
BTN_MOTOR_B   = 1   # pick motor B
BTN_MOTOR_C   = 2   # pick motor C
BTN_DIRECTION = 3   # your direction toggle/hold (0/1)
BTN_ENABLE    = 0   # reuse A as enable if that's your plan

BTN_SPEED_DOWN = 4  # slower
BTN_SPEED_UP   = 5  # faster

AXIS_A = 0  # velocity source for motor A
AXIS_B = 1  # velocity source for motor B
AXIS_C = 2  # velocity source for motor C

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

        # --- state for "remember last motor/vel" and button edges ---
        self.last_motor = 'A'                # 'A' | 'B' | 'C'
        self.last_vel   = {'A': 0, 'B': 0, 'C': 0}
        self.prev_buttons = []               # for edge detection
        self.off_us = 800                    # current speed setting

    def _btn(self, buttons, i):
        return (i < len(buttons)) and (buttons[i] == 1)

    def _axis(self, axes, i):
        return axes[i] if i < len(axes) else 0.0

    def _rose(self, buttons, i):
        prev = self.prev_buttons[i] if i < len(self.prev_buttons) else 0
        now  = 1 if (i < len(buttons) and buttons[i] == 1) else 0
        return prev == 0 and now == 1

    def on_joy(self, msg: Joy):
        # --- read velocities per motor from axes ---
        velA = vel_from_axis(self._axis(msg.axes, AXIS_A))
        velB = vel_from_axis(self._axis(msg.axes, AXIS_B))
        velC = vel_from_axis(self._axis(msg.axes, AXIS_C))

        # cache latest raw vels always
        self.last_vel['A'] = velA
        self.last_vel['B'] = velB
        self.last_vel['C'] = velC

        # --- select motor (if any button pressed); otherwise keep last ---
        motor = None
        if self._btn(msg.buttons, BTN_MOTOR_A):
            motor = 'A'
        elif self._btn(msg.buttons, BTN_MOTOR_B):
            motor = 'B'
        elif self._btn(msg.buttons, BTN_MOTOR_C):
            motor = 'C'
        else:
            motor = self.last_motor  # no button pressed -> use last motor

        # update last_motor if a selection is actively pressed
        if self._btn(msg.buttons, BTN_MOTOR_A) or \
           self._btn(msg.buttons, BTN_MOTOR_B) or \
           self._btn(msg.buttons, BTN_MOTOR_C):
            self.last_motor = motor

        # --- choose vel for the current (or last) motor ---
        vel_map = {'A': velA, 'B': velB, 'C': velC}
        vel = vel_map[motor] if motor in vel_map else 0
        # when no motor buttons are held, this still uses the
        # latest cached vel for last_motor (because we updated last_vel above)
        # If you instead want "freeze" behavior, replace with:
        # vel = self.last_vel[self.last_motor]

        # --- speed up/down with edges (one step per press) ---
        if self._rose(msg.buttons, BTN_SPEED_UP):
            self.off_us = max(MIN_OFF_US, self.off_us - 100)   # faster
        if self._rose(msg.buttons, BTN_SPEED_DOWN):
            self.off_us = min(MAX_OFF_US, self.off_us + 100)   # slower

        # --- other fields ---
        enable    = self._btn(msg.buttons, BTN_ENABLE)
        direction = 1 if self._btn(msg.buttons, BTN_DIRECTION) else 0

        # Publish (MotorCmd has only enable/direction/step_us)
        cmd = MotorCmd(enable=enable, direction=direction, step_us=self.off_us)
        self.pub.publish(cmd)

        # Helpful debug print to verify your “last motor/vel” behavior:
        self.get_logger().info(
            f"motor={motor} vel={vel} enable={enable} dir={direction} off_us={self.off_us}"
        )

        # update prev buttons for edge detection
        self.prev_buttons = list(msg.buttons)

def main():
    rclpy.init()
    rclpy.spin(JoyToStep())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
