"""Publish /cmd_vel so diff-drive math zeros one wheel velocity (other wheel moves).

Assumes TurtleBot3 OpenCR uses v_left = linear.x - (L/2)*angular.z,
v_right = linear.x + (L/2)*angular.z (standard diff drive). Burger L=0.160 m.
"""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


def main() -> None:
    rclpy.init()
    node = Node("tb3_single_wheel_test")
    node.declare_parameter("side", "left")
    node.declare_parameter("angular_z", 0.35)
    node.declare_parameter("wheel_separation_m", 0.160)
    node.declare_parameter("duration_sec", 3.0)
    node.declare_parameter("publish_hz", 20.0)

    side = str(node.get_parameter("side").value).lower().strip()
    omega = float(node.get_parameter("angular_z").value)
    L = float(node.get_parameter("wheel_separation_m").value)
    duration = float(node.get_parameter("duration_sec").value)
    hz = float(node.get_parameter("publish_hz").value)

    if side not in ("left", "right"):
        node.get_logger().error('Parameter "side" must be "left" or "right".')
        node.destroy_node()
        rclpy.shutdown()
        return

    half = L / 2.0
    twist = Twist()
    if side == "left":
        twist.linear.x = -half * omega
        twist.angular.z = omega
    else:
        twist.linear.x = half * omega
        twist.angular.z = omega

    pub = node.create_publisher(Twist, "/cmd_vel", 10)
    period = 1.0 / hz if hz > 0 else 0.05

    node.get_logger().info(
        f"Single-wheel test ({side}): linear.x={twist.linear.x:.4f} m/s, "
        f"angular.z={twist.angular.z:.4f} rad/s, L={L} m, {duration} s."
    )

    t_end = node.get_clock().now() + rclpy.duration.Duration(seconds=duration)
    while rclpy.ok() and node.get_clock().now() < t_end:
        pub.publish(twist)
        rclpy.spin_once(node, timeout_sec=period)

    pub.publish(Twist())
    node.get_logger().info("Sent zero velocity (stop).")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
