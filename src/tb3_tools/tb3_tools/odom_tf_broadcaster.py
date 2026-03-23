"""Broadcast TF from nav_msgs/Odometry.

Your system publishes /odom (nav_msgs/Odometry) but TF for the odom->base frame
may be missing. Nav2 and slam_toolbox need a connected TF tree.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped


class OdomTfBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("tb3_odom_tf_broadcaster")

        self.declare_parameter("odom_topic", "/odom")
        # Most TurtleBot3 stacks use parent "odom" and child "base_link".
        self.declare_parameter("parent_frame", "odom")
        self.declare_parameter("child_frame", "base_link")

        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._parent_frame = str(self.get_parameter("parent_frame").value)
        self._child_frame = str(self.get_parameter("child_frame").value)

        self._br = TransformBroadcaster(self)
        self.create_subscription(Odometry, self._odom_topic, self._cb, 10)
        self.get_logger().info(
            f"Broadcasting TF {self._parent_frame} -> {self._child_frame} from {self._odom_topic}"
        )

    def _cb(self, msg: Odometry) -> None:
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._parent_frame
        t.child_frame_id = self._child_frame

        t.transform.translation.x = float(msg.pose.pose.position.x)
        t.transform.translation.y = float(msg.pose.pose.position.y)
        t.transform.translation.z = float(msg.pose.pose.position.z)

        t.transform.rotation.x = float(msg.pose.pose.orientation.x)
        t.transform.rotation.y = float(msg.pose.pose.orientation.y)
        t.transform.rotation.z = float(msg.pose.pose.orientation.z)
        t.transform.rotation.w = float(msg.pose.pose.orientation.w)

        self._br.sendTransform(t)


def main() -> None:
    rclpy.init()
    node = OdomTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

