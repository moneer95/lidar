"""Publish forward motion on /cmd_vel for a fixed duration (TurtleBot3 + OpenCR test)."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from tb3_tools.motor_util import enable_motor_power


class Tb3ForwardTest(Node):
    def __init__(self) -> None:
        super().__init__("tb3_forward_test")
        self.declare_parameter("duration_sec", 2.0)
        self.declare_parameter("linear_x_m_s", 0.06)
        self.declare_parameter("publish_hz", 20.0)
        self.declare_parameter("stop_hold_sec", 2.0)
        self.declare_parameter("enable_motors", True)
        self.declare_parameter("motor_power_service", "/motor_power")

        duration = float(self.get_parameter("duration_sec").value)
        linear_x = float(self.get_parameter("linear_x_m_s").value)
        hz = float(self.get_parameter("publish_hz").value)
        stop_hold = float(self.get_parameter("stop_hold_sec").value)

        if bool(self.get_parameter("enable_motors").value):
            svc = str(self.get_parameter("motor_power_service").value)
            enable_motor_power(self, service_name=svc)

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
        t_stop = self.get_clock().now() + rclpy.duration.Duration(seconds=stop_hold)
        self.get_logger().info(
            f"Publishing zero cmd_vel for {stop_hold} s so OpenCR does not hold last speed."
        )
        while rclpy.ok() and self.get_clock().now() < t_stop:
            self._pub.publish(stop)
            rclpy.spin_once(self, timeout_sec=period)
        self.get_logger().info("Stop hold finished.")


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
