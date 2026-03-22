#!/usr/bin/env python3
"""Teleop TurtleBot3 with arrow keys (↑↓ linear, ←→ angular). Publishes /cmd_vel (ROS 2 Humble Twist)."""

from __future__ import annotations

import os
import select
import sys

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from tb3_tools.motor_util import enable_motor_power

if os.name != "nt":
    import termios
    import tty

BURGER_MAX_LIN_VEL = 0.22
BURGER_MAX_ANG_VEL = 2.84
WAFFLE_MAX_LIN_VEL = 0.26
WAFFLE_MAX_ANG_VEL = 1.82

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

MSG = """
TurtleBot3 arrow teleop (tb3_tools)
------------------------------------
  ↑   forward (more linear +)
  ↓   back     (more linear -)
  ←   turn CCW (more angular +)
  →   turn CW  (more angular -)

  space : stop (zero velocity)

  Same limits as turtlebot3_teleop; set TURTLEBOT3_MODEL=burger or waffle.
  On startup we call /motor_power on so wheels can move (like tb3_forward_test).

Ctrl-C to quit
"""


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def lin_limit(model: str, v: float) -> float:
    if model == "burger":
        return clamp(v, -BURGER_MAX_LIN_VEL, BURGER_MAX_LIN_VEL)
    return clamp(v, -WAFFLE_MAX_LIN_VEL, WAFFLE_MAX_LIN_VEL)


def ang_limit(model: str, v: float) -> float:
    if model == "burger":
        return clamp(v, -BURGER_MAX_ANG_VEL, BURGER_MAX_ANG_VEL)
    return clamp(v, -WAFFLE_MAX_ANG_VEL, WAFFLE_MAX_ANG_VEL)


def make_simple_profile(output_vel: float, input_vel: float, slop: float) -> float:
    if input_vel > output_vel:
        return min(input_vel, output_vel + slop)
    if input_vel < output_vel:
        return max(input_vel, output_vel - slop)
    return input_vel


def read_key(settings) -> str:
    """Return '', 'UP','DOWN','LEFT','RIGHT', ' ', or '\\x03' (Ctrl-C)."""
    if os.name == "nt":
        return ""
    tty.setraw(sys.stdin.fileno())
    try:
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if not rlist:
            return ""
        c = sys.stdin.read(1)
        if c in (" ", "\x03"):
            return c
        if c != "\x1b":
            return ""
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
        if not rlist:
            return ""
        c2 = sys.stdin.read(1)
        if c2 == "[":
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not rlist:
                return ""
            c3 = sys.stdin.read(1)
            if c3 == "A":
                return "UP"
            if c3 == "B":
                return "DOWN"
            if c3 == "D":
                return "LEFT"
            if c3 == "C":
                return "RIGHT"
        elif c2 == "O":
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if not rlist:
                return ""
            c3 = sys.stdin.read(1)
            if c3 == "A":
                return "UP"
            if c3 == "B":
                return "DOWN"
            if c3 == "D":
                return "LEFT"
            if c3 == "C":
                return "RIGHT"
        return ""
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


def main() -> None:
    settings = None
    if os.name != "nt":
        settings = termios.tcgetattr(sys.stdin)

    model = os.environ.get("TURTLEBOT3_MODEL", "burger").strip().lower()
    if model not in ("burger", "waffle", "waffle_pi"):
        print("Set TURTLEBOT3_MODEL to burger, waffle, or waffle_pi.", file=sys.stderr)
        sys.exit(1)
    if model == "waffle_pi":
        model = "waffle"

    rclpy.init()
    node = Node("tb3_arrow_teleop")
    node.declare_parameter("enable_motors", True)
    node.declare_parameter("motor_power_service", "/motor_power")

    qos = QoSProfile(depth=10)
    pub = node.create_publisher(Twist, "/cmd_vel", qos)

    if bool(node.get_parameter("enable_motors").value):
        svc = str(node.get_parameter("motor_power_service").value)
        if enable_motor_power(node, service_name=svc):
            print("Motors enabled (/motor_power). If wheels still ignore cmd_vel, call the service manually.")
        else:
            print(
                "WARNING: /motor_power not available or failed. With bringup running, try:\n"
                "  ros2 service call /motor_power std_srvs/srv/SetBool \"{data: true}\""
            )

    target_linear = 0.0
    target_angular = 0.0
    control_linear = 0.0
    control_angular = 0.0
    status = 0

    print(MSG)
    print(f"model={model}\tlinear={target_linear}\tangular={target_angular}")

    try:
        while rclpy.ok():
            key = read_key(settings)
            if key == "UP":
                target_linear = lin_limit(model, target_linear + LIN_VEL_STEP_SIZE)
                status += 1
                print(f"linear={target_linear:.3f}\tangular={target_angular:.3f}")
            elif key == "DOWN":
                target_linear = lin_limit(model, target_linear - LIN_VEL_STEP_SIZE)
                status += 1
                print(f"linear={target_linear:.3f}\tangular={target_angular:.3f}")
            elif key == "LEFT":
                target_angular = ang_limit(model, target_angular + ANG_VEL_STEP_SIZE)
                status += 1
                print(f"linear={target_linear:.3f}\tangular={target_angular:.3f}")
            elif key == "RIGHT":
                target_angular = ang_limit(model, target_angular - ANG_VEL_STEP_SIZE)
                status += 1
                print(f"linear={target_linear:.3f}\tangular={target_angular:.3f}")
            elif key == " ":
                target_linear = 0.0
                control_linear = 0.0
                target_angular = 0.0
                control_angular = 0.0
                print(f"linear={target_linear:.3f}\tangular={target_angular:.3f} (stop)")
            elif key == "\x03":
                break

            if status >= 20:
                print(MSG)
                status = 0

            control_linear = make_simple_profile(
                control_linear, target_linear, LIN_VEL_STEP_SIZE / 2.0
            )
            control_angular = make_simple_profile(
                control_angular, target_angular, ANG_VEL_STEP_SIZE / 2.0
            )

            twist = Twist()
            twist.linear.x = control_linear
            twist.angular.z = control_angular
            pub.publish(twist)
    finally:
        pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if os.name != "nt" and settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == "__main__":
    main()
