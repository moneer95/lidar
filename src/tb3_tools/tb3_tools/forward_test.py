"""Publish forward motion on /cmd_vel for a fixed duration (TurtleBot3 + OpenCR test)."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class Tb3ForwardTest(Node):
    def __init__(self) -> None:
        super().__init__("tb3_forward_test")
        self.declare_parameter("duration_sec", 2.0)
        self.declare_parameter("linear_x_m_s", 0.08)
        self.declare_parameter("publish_hz", 20.0)

        duration = float(self.get_parameter("duration_sec").value)
        linear_x = float(self.get_parameter("linear_x_m_s").value)
        hz = float(self.get_parameter("publish_hz").value)

        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        period = 1.0 / hz if hz > 0 else 0.05
        twist = Twist()
        twist.linear.x = linear_x

        self.get_logger().info(
            f"Driving forward: linear.x={linear_x} m/s for {duration} s (then stop)."
        )

        t_end = self.get_clock().now() + rclpy.duration.Duration(seconds=duration)
        while self.get_clock().now() < t_end:
            self._pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=period)

        stop = Twist()
        self._pub.publish(stop)
        self.get_logger().info("Sent zero velocity (stop).")


def main() -> None:
    rclpy.init()
    node = None
    try:
        node = Tb3ForwardTest()
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
