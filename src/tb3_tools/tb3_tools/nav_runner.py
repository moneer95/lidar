"""Generic TurtleBot3 navigation runner.

This node handles ROS I/O (subscribe /scan, publish /cmd_vel) and delegates
decision logic to plugin algorithms loaded from the external algorithms folder.
"""

from __future__ import annotations

import os
from pathlib import Path

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from tb3_tools.algorithm_api import LidarObservation, NavigationAlgorithm
from tb3_tools.algorithm_loader import discover_algorithms
from tb3_tools.motor_util import enable_motor_power


DEFAULT_ALGO_PARAMS = {
    "linear_x": 0.05,
    "max_angular_z": 0.8,
    "kp": 1.0,
    "safety_distance_m": 0.25,
    "fov_deg": 120.0,
    "forward_lidar_angle_deg": 120.0,
}


class Tb3NavRunner(Node):
    def __init__(self) -> None:
        # Allow algorithm-specific params from --params-file without pre-declaration.
        super().__init__("tb3_nav", automatically_declare_parameters_from_overrides=True)
        self.declare_parameter("algorithm", "")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("control_hz", 20.0)
        self.declare_parameter("enable_motors", True)
        self.declare_parameter("motor_power_service", "/motor_power")
        self.declare_parameter("invert_drive", True)
        self.declare_parameter("algorithms_dir", "")

        algorithms_dir = self._resolve_algorithms_dir()
        self.get_logger().info(f"Loading algorithms from: {algorithms_dir}")

        registry = discover_algorithms(algorithms_dir)
        if not registry:
            raise ValueError(
                f"No navigation algorithms found in {algorithms_dir}. "
                "Add a plugin .py file there implementing NavigationAlgorithm."
            )

        algo_name = str(self.get_parameter("algorithm").value).strip()
        if not algo_name:
            algo_name = sorted(registry.keys())[0]
            self.get_logger().info(
                f'No algorithm specified; auto-selecting "{algo_name}".'
            )
        algo_cls = registry.get(algo_name)
        if algo_cls is None:
            valid = ", ".join(sorted(registry.keys()))
            raise ValueError(
                f'Unknown algorithm "{algo_name}". Valid options: {valid}'
            )
        self._algorithm: NavigationAlgorithm = algo_cls()
        self._algorithm.configure(self._collect_algorithm_params())

        scan_topic = str(self.get_parameter("scan_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)

        if bool(self.get_parameter("enable_motors").value):
            enable_motor_power(
                self,
                service_name=str(self.get_parameter("motor_power_service").value),
            )

        self._pub = self.create_publisher(Twist, cmd_topic, 10)
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, scan_qos
        )
        self._last_scan: LaserScan | None = None

        hz = float(self.get_parameter("control_hz").value)
        period = 1.0 / hz if hz > 0 else 0.05
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f'tb3_nav ready: algorithm="{algo_name}" scan={scan_topic} -> {cmd_topic}'
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self._last_scan = msg

    def _on_timer(self) -> None:
        twist = Twist()
        scan = self._last_scan
        if scan is None:
            self._pub.publish(twist)
            return

        obs = self._to_observation(scan)
        cmd = self._algorithm.compute(obs)
        v = float(cmd.linear_x_m_s)
        w = float(cmd.angular_z_rad_s)
        if bool(self.get_parameter("invert_drive").value):
            v = -v
            w = -w
        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self._pub.publish(twist)

    def stop(self) -> None:
        self._pub.publish(Twist())

    def _to_observation(self, scan: LaserScan) -> LidarObservation:
        count = len(scan.ranges)
        angles = [
            float(scan.angle_min) + float(i) * float(scan.angle_increment)
            for i in range(count)
        ]
        ranges = [float(r) for r in scan.ranges]
        return LidarObservation(
            ranges_m=ranges,
            angles_rad=angles,
            angle_min_rad=float(scan.angle_min),
            angle_max_rad=float(scan.angle_max),
            angle_increment_rad=float(scan.angle_increment),
            range_min_m=float(scan.range_min),
            range_max_m=float(scan.range_max),
            scan_time_s=float(scan.scan_time),
        )

    def _collect_algorithm_params(self) -> dict:
        """Collect algorithm config from params with `algo.` prefix."""
        names = self.list_parameters(["algo"], 10).names
        cfg = dict(DEFAULT_ALGO_PARAMS)
        for full_name in names:
            key = full_name.removeprefix("algo.")
            cfg[key] = self.get_parameter(full_name).value
        return cfg

    def _resolve_algorithms_dir(self) -> Path:
        """Resolve external algorithm directory with clear precedence."""
        param_dir = str(self.get_parameter("algorithms_dir").value).strip()
        if param_dir:
            return Path(param_dir).expanduser().resolve()

        env_dir = os.environ.get("TB3_ALGORITHMS_DIR", "").strip()
        if env_dir:
            return Path(env_dir).expanduser().resolve()

        # Workspace default: <repo>/src/algorithms
        return Path(__file__).resolve().parents[2] / "algorithms"


def main() -> None:
    rclpy.init()
    node = Tb3NavRunner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

