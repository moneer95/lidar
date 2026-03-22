"""TurtleBot3: enable wheel torque via motor_power SetBool service."""

import rclpy
from std_srvs.srv import SetBool


def enable_motor_power(
    node,
    service_name: str = "/motor_power",
    timeout_sec: float = 5.0,
) -> bool:
    """Return True if service called successfully or was skipped as unavailable."""
    cli = node.create_client(SetBool, service_name)
    if not cli.wait_for_service(timeout_sec=timeout_sec):
        node.get_logger().warn(
            f"Service {service_name} not available — enable torque manually if needed."
        )
        return False
    req = SetBool.Request()
    req.data = True
    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_sec)
    if not future.done():
        node.get_logger().error("motor_power service call timed out.")
        return False
    res = future.result()
    if res is None or not res.success:
        msg = res.message if res else "no response"
        node.get_logger().error(f"motor_power failed: {msg}")
        return False
    node.get_logger().info("Motors enabled (motor_power on).")
    return True
