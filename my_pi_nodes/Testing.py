"""
test.py — interactive “virtual controller” + introspection for Control_Test

Run:
  python test.py

Commands:
  v <vx> <vy> <vz>      set virtual joystick vector (unitless)
  dz <deadzone>         set deadzone (applied in this test harness, not in Control_Test)
  gain <g>              set cart_gain passed into ctrl.step() (scales v -> mm/s inside qdot_from_v)
  dt <seconds>          set timestep dt
  step [n]              step n times (default 1)
  run <seconds>         run for given seconds (using current dt)
  stop                  set v = [0,0,0]
  show                  show key state (thetas, tip, qdot, off_us, cond(J))
  show J                print Jacobian
  show all              print everything
  reset [t1 t2 t3]      reset thetas (deg), default [0,0,0]
  help                  print help
  quit                  exit

Notes:
- This uses your Control_Test.dynamics.step(dt, v, gain, deadzone=0.0) API.
- We apply deadzone in this file, so deadzone inside Control_Test is disabled (deadzone=0.0).
"""

import math
import time
import numpy as np

# ---- Import your control core ----
# Option A: if test.py is in same folder as Control_Test.py
from Control_Test import dynamics

# If you instead want to import from a package, do something like:
# from my_pi_nodes.Control_Test import dynamics


# ---- Motor mapping constants (match your ROS node) ----
DEG_PER_STEP = 0.045
ON_US = 5.0
OFF_US_MIN = 1000.0
OFF_US_MAX = 2800.0

MAX_STEP_FREQ = 1e6 / (ON_US + OFF_US_MIN)
MAX_QDOT = 0.4 * MAX_STEP_FREQ * DEG_PER_STEP  # deg/s safety cap


def limit_qdot(qdot: np.ndarray) -> np.ndarray:
    qdot = np.asarray(qdot, dtype=float)
    max_abs = float(np.max(np.abs(qdot))) if qdot.size else 0.0
    if max_abs > MAX_QDOT and max_abs > 1e-12:
        qdot = qdot * (MAX_QDOT / max_abs)
    return qdot


def qdot_to_off_us(qdot: np.ndarray) -> np.ndarray:
    qdot = np.asarray(qdot, dtype=float)
    out = np.zeros(3, dtype=float)

    for i, qd in enumerate(qdot):
        if abs(qd) < 1e-6:
            out[i] = 0.0
            continue
        s = min(abs(qd) / MAX_QDOT, 1.0)
        off_us = OFF_US_MAX - s * (OFF_US_MAX - OFF_US_MIN)
        out[i] = math.copysign(off_us, qd)

    return out


def cond_number(J: np.ndarray) -> float:
    # Condition number based on singular values
    U, S, Vt = np.linalg.svd(J)
    if S[-1] < 1e-12:
        return float("inf")
    return float(S[0] / S[-1])


def pretty(v: np.ndarray, nd=4) -> str:
    v = np.asarray(v, dtype=float)
    return "[" + ", ".join(f"{x:.{nd}f}" for x in v.tolist()) + "]"


HELP = """
Commands:
  v <vx> <vy> <vz>      set virtual joystick vector (unitless)
  dz <deadzone>         set deadzone (applied here)
  gain <g>              set cart_gain passed into ctrl.step()
  dt <seconds>          set timestep dt
  step [n]              step n times (default 1)
  run <seconds>         run for given seconds (using current dt)
  stop                  set v = [0,0,0]
  show                  show key state
  show J                print Jacobian
  show all              print everything
  reset [t1 t2 t3]      reset thetas (deg), default [0,0,0]
  help                  print this help
  quit                  exit
"""


def main():
    ctrl = dynamics([0.0, 0.0, 0.0])

    dt = 0.02
    cart_gain = 5.0
    deadzone = 0.10
    v_cmd = np.zeros(3, dtype=float)

    last_qdot = np.zeros(3, dtype=float)

    def apply_deadzone(v: np.ndarray) -> np.ndarray:
        v = np.asarray(v, dtype=float).copy()
        v[np.abs(v) < deadzone] = 0.0
        return v

    def step_once():
        nonlocal last_qdot
        v = apply_deadzone(v_cmd)

        if np.linalg.norm(v) < 1e-9:
            last_qdot = np.zeros(3, dtype=float)
            return

        # Use your core step() (deadzone disabled inside Control_Test)
        thetas, qdot = ctrl.step(dt, v=v, gain=cart_gain, deadzone=0.0)

        # Enforce physical qdot limit safely by undo/redo integration if needed
        qdot_limited = limit_qdot(qdot)
        if np.any(np.abs(qdot_limited - qdot) > 1e-12):
            ctrl.thetas = ctrl.thetas - qdot * dt + qdot_limited * dt
            qdot = qdot_limited

        last_qdot = qdot

    def show(mode="key"):
        tip = ctrl.fk(ctrl.thetas)
        J = ctrl.numJ()
        condJ = cond_number(J)
        off_us = qdot_to_off_us(last_qdot)

        if mode in ("key", "all"):
            print(f"dt={dt:.4f}  cart_gain={cart_gain:.3f}  deadzone={deadzone:.3f}")
            print(f"v_cmd={pretty(v_cmd, 3)}  v_eff={pretty(apply_deadzone(v_cmd), 3)}")
            print(f"thetas(deg)={pretty(ctrl.thetas, 3)}")
            print(f"tip(mm)={pretty(tip, 3)}")
            print(f"qdot(deg/s)={pretty(last_qdot, 4)}  |qdot|max={np.max(np.abs(last_qdot)):.4f}  cond(J)={condJ:.2f}")
            print(f"off_us={pretty(off_us, 1)}")

        if mode in ("J", "all"):
            print("J (mm/deg):")
            print(np.array2string(J, precision=6, suppress_small=True))

    print("Interactive delta test harness. Type 'help' for commands.")
    while True:
        try:
            line = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\nquit")
            break

        if not line:
            continue

        parts = line.split()
        cmd = parts[0].lower()

        try:
            if cmd in ("quit", "exit", "q"):
                break

            elif cmd == "help":
                print(HELP)

            elif cmd == "v":
                if len(parts) != 4:
                    print("usage: v <vx> <vy> <vz>")
                    continue
                v_cmd[:] = [float(parts[1]), float(parts[2]), float(parts[3])]

            elif cmd == "stop":
                v_cmd[:] = 0.0

            elif cmd == "dz":
                if len(parts) != 2:
                    print("usage: dz <deadzone>")
                    continue
                deadzone = max(0.0, float(parts[1]))

            elif cmd == "gain":
                if len(parts) != 2:
                    print("usage: gain <g>")
                    continue
                cart_gain = float(parts[1])

            elif cmd == "dt":
                if len(parts) != 2:
                    print("usage: dt <seconds>")
                    continue
                dt = float(parts[1])

            elif cmd == "step":
                n = int(parts[1]) if len(parts) > 1 else 1
                for _ in range(n):
                    step_once()

            elif cmd == "run":
                if len(parts) != 2:
                    print("usage: run <seconds>")
                    continue
                T = float(parts[1])
                n = max(1, int(round(T / dt)))
                for _ in range(n):
                    step_once()

            elif cmd == "show":
                if len(parts) == 1:
                    show("key")
                elif parts[1].lower() == "j":
                    show("J")
                elif parts[1].lower() == "all":
                    show("all")
                else:
                    print("usage: show | show J | show all")

            elif cmd == "reset":
                if len(parts) == 1:
                    ctrl.thetas[:] = [0.0, 0.0, 0.0]
                elif len(parts) == 4:
                    ctrl.thetas[:] = [float(parts[1]), float(parts[2]), float(parts[3])]
                else:
                    print("usage: reset [t1 t2 t3]")
                    continue
                last_qdot[:] = 0.0

            else:
                print("Unknown command. Type 'help'.")

        except Exception as e:
            print(f"Error: {e}")


if __name__ == "__main__":
    main()