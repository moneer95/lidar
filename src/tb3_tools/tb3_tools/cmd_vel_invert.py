"""Subscribe to cmd_vel (Twist), optionally negate linear.x / angular.z, republish."""

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelInvert(Node):
    def __init__(self) -> None:
        super().__init__("tb3_cmd_vel_invert")
        self.declare_parameter("input_topic", "/cmd_vel_raw")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("invert", True)

        in_topic = str(self.get_parameter("input_topic").value)
        out_topic = str(self.get_parameter("output_topic").value)
        self._invert = bool(self.get_parameter("invert").value)

        self._pub = self.create_publisher(Twist, out_topic, 10)
        self.create_subscription(Twist, in_topic, self._cb, 10)
        self.get_logger().info(
            f"cmd_vel invert: {in_topic} -> {out_topic}, invert={self._invert}"
        )

    def _cb(self, msg: Twist) -> None:
        out = Twist()
        if self._invert:
            out.linear.x = -msg.linear.x
            out.linear.y = -msg.linear.y
            out.linear.z = msg.linear.z
            out.angular.x = msg.angular.x
            out.angular.y = msg.angular.y
            out.angular.z = -msg.angular.z
        else:
            out = msg
        self._pub.publish(out)


def main() -> None:
    rclpy.init()
    node = CmdVelInvert()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        z = Twist()
        node._pub.publish(z)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
