# Copilot Instructions for DeltaRobot

This is a **ROS 2 Python package** (`my_pi_nodes`) controlling a delta robot platform. The system integrates joystick input, motor control via serial communication, and kinematics/dynamics computation.

## Architecture Overview

### Core Components

1. **Control.py** – Delta robot kinematics & dynamics
   - Forward kinematics: `delta_calcForward(theta1, theta2, theta3)` → Cartesian position
   - Inverse kinematics: `delta_calcInverse(x, y, z)` → joint angles
   - `dynamics` class: maintains state (joint angles), Jacobian computation, workspace limits
   - `ctrl_mov`: Cartesian-space position controller (target x,y,z → qdot)
   - `theta_mov`: joint-space controller (target θ1,θ2,θ3 → qdot)

2. **controller.py** – Joystick input bridge
   - Reads gamepad via pygame at 50 Hz
   - Publishes `sensor_msgs/Joy` on `joy` topic
   - Optional: includes hat axes as continuous inputs (configurable param)

3. **MotorCmd.py** – Main control loop (named `DeltaControl` node)
   - Subscribes: `joy`, `move_target` (Cartesian), `theta_target` (joint angles)
   - Publishes: `motor_cmd` (3 motor commands), `tip_position` (Cartesian feedback)
   - Updates `ctrl_mov` / `theta_mov` controllers each 50 Hz cycle
   - Converts joystick axes to Cartesian velocity (joy → tip_vel)

4. **serial_bridge.py** – Hardware communication
   - Bidirectional serial over `/dev/ttyUSB0` (115200 baud, configurable)
   - Expects 3 motor commands: `cmdA,cmdB,cmdC\n` (CSV format)
   - Subscribes: `motor_cmd` (`Float32MultiArray` with exactly 3 elements)
   - Non-blocking reads from Arduino for debugging

5. **tip_plotter.py** – Live 3D visualization
   - Displays end-effector trajectory in matplotlib 3D plot
   - Subscribes: `tip_position` topic
   - Updates at 20 Hz

### Data Flow

```
[Joystick] → controller.py (Joy topic)
                              ↓
[MotorCmd.py] (main loop) ← joy
    ↓ (computes qdot via dynamics/Jacobian)
[motor_cmd topic] → serial_bridge.py
                    ↓
              [Arduino via Serial]
                    ↓
              [tip_position topic] → tip_plotter.py
```

## Developer Workflows

### Build & Test (ament_python)
```bash
# In workspace root
colcon build --packages-select my_pi_nodes
colcon test --packages-select my_pi_nodes
```

### Lint Checks (ROS 2 standard)
- **flake8**: Code style (PEP8)
- **pep257**: Docstring conventions
- **copyright**: License header validation (currently skipped in test_copyright.py)

Tests live in `test/` and run via pytest + ament integration.

### Run Nodes
```bash
# Terminal 1: Joystick reader
ros2 run my_pi_nodes controller

# Terminal 2: Main control loop
ros2 run my_pi_nodes MotorCmd

# Terminal 3: Serial communication
ros2 run my_pi_nodes serial_bridge --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200

# Terminal 4: Visualization (optional)
ros2 run my_pi_nodes tip_plotter
```

### Key ROS Parameters
- `controller.include_hats_as_axes` (bool): Append hat (D-pad) values to axes list
- `serial_bridge.port` (str): Serial port (default `/dev/ttyUSB0`)
- `serial_bridge.baud` (int): Baud rate (default 115200)
- `serial_bridge.format` (str): Must be `'csv'` (forced)

### Topic Reference
| Topic | Type | Publisher | Subscriber | Purpose |
|-------|------|-----------|-----------|---------|
| `joy` | `sensor_msgs/Joy` | controller.py | MotorCmd.py | Gamepad input |
| `motor_cmd` | `Float32MultiArray` | MotorCmd.py | serial_bridge.py | Motor commands [cmdA, cmdB, cmdC] |
| `tip_position` | `Float32MultiArray` | MotorCmd.py (from dynamics) | tip_plotter.py | End-effector Cartesian [x, y, z] in mm |
| `move_target` | `geometry_msgs/Point` | (external) | MotorCmd.py | Cartesian move goal |
| `theta_target` | `Float32MultiArray` | (external) | MotorCmd.py | Joint angle goal [θ1, θ2, θ3] in deg |

## Project-Specific Patterns

### Kinematics Constants (Control.py)
All geometry parameters hard-coded at module level:
```python
e, f = 45, 80           # Platform dimensions (mm)
re, rf = 272, 235       # Arm segment lengths (mm)
```
Modify these if robot hardware changes.

### Control Gains & Limits
Located in **MotorCmd.py** and **Control.py**:
- `DEG_PER_STEP = 0.045`: stepper motor microstep resolution
- `MAX_STEP_FREQ`: maximum stepper frequency (motor hardware limit)
- `kp=0.8` (cartesian), `kp=2.0` (joint): P-controller gains
- `max_tip_speed=30 mm/s`: velocity limit
- `max_qdot`: joint velocity limit

**Workspace limits** in `dynamics.limits`:
```python
limits = [[-160.8, 160.8], [-160.8, 160.8], [-370.0, -100.0]]  # [θ1, θ2, θ3] ranges
```
Used in `qdot_from_v()` to prevent exceeding workspace boundaries.

### Jacobian-Based Inverse Velocity (Damped Least Squares)
In `Control.py` → `dynamics.qdot_from_v()`:
- Computes Jacobian numerically (3-point difference, h=0.1)
- Uses SVD + damped pseudo-inverse for robustness
- Condition number check: increases damping if J is ill-conditioned (λ adapts based on κ)
- Applies workspace boundary checking before final qdot computation

### Status Codes
Functions return `(status, data...)` tuples:
- `0`: Success
- `-1`: Failure (e.g., unreachable position, singular Jacobian)

Always check status before using returned data.

### Serial Communication Format
Arduino receives: `"cmdA,cmdB,cmdC\n"` (CSV, 6 decimal places)
Expected response: ack or telemetry (non-blocking read, logged for debugging)

## Integration Points & External Dependencies

- **ROS 2** (rclpy): Node creation, timers, subscriptions, publishers
- **numpy**: Matrix operations (Jacobian, SVD, linear algebra)
- **pygame**: Joystick input (requires SDL with dummy video driver on headless systems)
- **pyserial**: Serial port communication to Arduino
- **matplotlib**: 3D visualization (tip_plotter.py, optional)

### Headless Operation
`controller.py` sets `SDL_VIDEODRIVER="dummy"` to run without display—critical for Raspberry Pi deployment.

## Code Quality Notes

- **Import conventions**: No wildcard imports; explicit ROS message types
- **Numpy usage**: Prefer `.array()` with dtype specification for precision
- **Error handling**: Check return status codes; graceful failures preferred
- **Logging**: Use `self.get_logger().{info,warn,error}()` for all diagnostics
- **Type hints**: Present in function signatures (float, int, np.ndarray); used for clarity
- **Testing**: ROS 2 standard linters (flake8, pep257); copyright header currently skipped
