"""Steer TurtleBot3 toward the center of the widest clear gap in the forward LiDAR sector.

Interprets "gap between two obstacles" as the largest contiguous angular region where ranges
exceed a clearance distance; publishes /cmd_vel to align with that region's bisector.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

from geometry_msgs.msg import Twist
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan

from tb3_tools.motor_util import enable_motor_power

# CLI often passes integers (e.g. -p fov_deg:=80); avoid INTEGER vs DOUBLE errors.
_PARAM_NUM = ParameterDescriptor(dynamic_typing=True)


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class GapNav(Node):
    def __init__(self) -> None:
        super().__init__("tb3_gap_nav")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("control_hz", 20.0, _PARAM_NUM)

        # Forward sector (symmetric around body-frame angle 0 = robot +X forward)
        self.declare_parameter("fov_deg", 120.0, _PARAM_NUM)
        # Raw LaserScan angle (deg) that points along robot +X; subtracted from each beam.
        # Example: if the driver’s 0° is to the side but forward is at +120° in that frame, set 120.
        self.declare_parameter("forward_lidar_angle_deg", 120.0, _PARAM_NUM)

        # Ray is "free" if range >= this (m); else treated as obstacle for gap detection
        self.declare_parameter("clear_distance_m", 0.32, _PARAM_NUM)
        # Ignore readings beyond this when classifying (match lidar range_max, e.g. 0.4 m)
        self.declare_parameter("max_obstacle_range_m", 0.4, _PARAM_NUM)

        self.declare_parameter("linear_x", 0.04, _PARAM_NUM)
        self.declare_parameter("max_angular_z", 0.65, _PARAM_NUM)
        self.declare_parameter("angular_gain", 1.2, _PARAM_NUM)

        # Slow down when turning hard
        self.declare_parameter("align_threshold_rad", 0.35, _PARAM_NUM)
        self.declare_parameter("linear_scale_when_misaligned", 0.25, _PARAM_NUM)

        # Stop if anything this close in a narrow safety wedge
        self.declare_parameter("emergency_stop_m", 0.22, _PARAM_NUM)
        self.declare_parameter("safety_fov_deg", 70.0, _PARAM_NUM)

        self.declare_parameter("enable_motors", True)
        self.declare_parameter("motor_power_service", "/motor_power")
        # If robot +X / cmd_vel forward is opposite real drive: set true to negate linear + angular.
        # Default to True so "invert" behavior is enabled out-of-the-box.
        self.declare_parameter("invert_drive", True)

        # Auto-config: derive nav FOV + range thresholds from the incoming LaserScan
        # so `tb3_gap_nav` matches your LiDAR driver YAML without separate params.
        self.declare_parameter("auto_config_from_scan", True)
        self.declare_parameter("clear_distance_fraction", 0.8)
        self.declare_parameter("emergency_stop_fraction", 0.55)
        self._autoconfig_done = False

        scan_topic = str(self.get_parameter("scan_topic").value)
        cmd_topic = str(self.get_parameter("cmd_vel_topic").value)

        if bool(self.get_parameter("enable_motors").value):
            enable_motor_power(
                self,
                service_name=str(self.get_parameter("motor_power_service").value),
            )

        self._pub = self.create_publisher(Twist, cmd_topic, 10)
        # Match typical /scan publishers (e.g. YDLidar): BEST_EFFORT — default RELIABLE gets no data.
        _scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub = self.create_subscription(
            LaserScan, scan_topic, self._on_scan, _scan_qos
        )
        self._last_scan: Optional[LaserScan] = None
        self._last_emergency_warn = None

        hz = float(self.get_parameter("control_hz").value)
        period = 1.0 / hz if hz > 0 else 0.05
        self.create_timer(period, self._on_timer)

        off_deg = float(self.get_parameter("forward_lidar_angle_deg").value)
        inv = bool(self.get_parameter("invert_drive").value)
        self.get_logger().info(
            f"Gap nav: scan={scan_topic} -> {cmd_topic}; "
            f"forward_lidar_angle_deg={off_deg} (0 rad in logic = robot +X); "
            f"invert_drive={inv}. Widest clear sector in forward FOV; steer to its center."
        )

    def _body_frame_angles(self, scan: LaserScan) -> List[float]:
        """Each beam angle in a frame where 0 = robot +X (after subtracting forward_lidar_angle_deg)."""
        off = math.radians(float(self.get_parameter("forward_lidar_angle_deg").value))
        n = len(scan.ranges)
        return [
            scan.angle_min + float(i) * scan.angle_increment - off for i in range(n)
        ]

    def _on_scan(self, msg: LaserScan) -> None:
        if bool(self.get_parameter("auto_config_from_scan").value) and not self._autoconfig_done:
            # LaserScan stores angles in radians. We map robot "forward" to the midpoint
            # of the configured arc, and set thresholds from scan.range_max.
            angle_min_deg = math.degrees(float(msg.angle_min))
            angle_max_deg = math.degrees(float(msg.angle_max))
            fov = abs(angle_max_deg - angle_min_deg)
            center = (angle_min_deg + angle_max_deg) / 2.0

            r_max = float(msg.range_max)
            if math.isfinite(r_max) and r_max > 0.0 and fov > 0.0:
                clear_frac = float(self.get_parameter("clear_distance_fraction").value)
                stop_frac = float(self.get_parameter("emergency_stop_fraction").value)

                self.set_parameters(
                    [
                        rclpy.parameter.Parameter(
                            "fov_deg",
                            rclpy.Parameter.Type.DOUBLE,
                            float(fov),
                        ),
                        rclpy.parameter.Parameter(
                            "forward_lidar_angle_deg",
                            rclpy.Parameter.Type.DOUBLE,
                            float(center),
                        ),
                        rclpy.parameter.Parameter(
                            "max_obstacle_range_m",
                            rclpy.Parameter.Type.DOUBLE,
                            r_max,
                        ),
                        rclpy.parameter.Parameter(
                            "clear_distance_m",
                            rclpy.Parameter.Type.DOUBLE,
                            r_max * clear_frac,
                        ),
                        rclpy.parameter.Parameter(
                            "emergency_stop_m",
                            rclpy.Parameter.Type.DOUBLE,
                            r_max * stop_frac,
                        ),
                    ]
                )
                self.get_logger().info(
                    "Gap nav auto-config from /scan: "
                    f"fov_deg={fov:.1f}, forward_lidar_angle_deg={center:.1f}, "
                    f"range_max={r_max:.2f}m -> clear_distance_m={r_max * clear_frac:.2f}m, "
                    f"emergency_stop_m={r_max * stop_frac:.2f}m"
                )
                self._autoconfig_done = True
        self._last_scan = msg

    def _classify_free(
        self,
        scan: LaserScan,
        r: float,
        clear_m: float,
        max_obs_m: float,
    ) -> bool:
        if math.isnan(r) or math.isinf(r):
            return True
        if r < scan.range_min or r > scan.range_max:
            return False
        if r > max_obs_m:
            return True
        return r >= clear_m

    def _best_gap_center(
        self, scan: LaserScan, fov_rad: float
    ) -> Optional[Tuple[float, float]]:
        """Return (center_angle_rad, angular_width_rad) of best gap, or None."""
        angles = self._body_frame_angles(scan)
        n = len(angles)
        if n == 0:
            return None

        clear_m = float(self.get_parameter("clear_distance_m").value)
        max_obs = float(self.get_parameter("max_obstacle_range_m").value)

        free: List[bool] = []
        for i, ang in enumerate(angles):
            if abs(_wrap_pi(ang)) > fov_rad / 2.0:
                free.append(False)
                continue
            free.append(self._classify_free(scan, scan.ranges[i], clear_m, max_obs))

        best: Optional[Tuple[float, float, float]] = None  # score, center, width

        i = 0
        while i < n:
            if not free[i]:
                i += 1
                continue
            j = i
            while j < n and free[j]:
                j += 1
            # segment [i, j)
            a0 = angles[i]
            a1 = angles[j - 1]
            width = abs(a1 - a0) + abs(scan.angle_increment)
            center = 0.5 * (a0 + a1)
            # Prefer wide gaps slightly over off-center narrow ones
            score = width - 0.4 * abs(_wrap_pi(center))
            if best is None or score > best[0]:
                best = (score, center, width)
            i = j

        if best is None:
            return None
        return (best[1], best[2])

    def _min_range_forward(self, scan: LaserScan, fov_rad: float) -> float:
        angles = self._body_frame_angles(scan)
        m = float("inf")
        for i, ang in enumerate(angles):
            if abs(_wrap_pi(ang)) > fov_rad / 2.0:
                continue
            r = scan.ranges[i]
            if math.isnan(r) or math.isinf(r):
                continue
            if r < scan.range_min or r > scan.range_max:
                continue
            m = min(m, r)
        return m

    def _on_timer(self) -> None:
        scan = self._last_scan
        twist = Twist()
        if scan is None:
            self._pub.publish(twist)
            return

        fov = math.radians(float(self.get_parameter("fov_deg").value))
        safety_fov = math.radians(float(self.get_parameter("safety_fov_deg").value))
        emergency = float(self.get_parameter("emergency_stop_m").value)

        d_close = self._min_range_forward(scan, safety_fov)
        if d_close < emergency:
            now = self.get_clock().now()
            if self._last_emergency_warn is None or (
                now - self._last_emergency_warn
            ).nanoseconds >= 1_000_000_000:
                self.get_logger().warn(
                    f"Emergency stop: obstacle at {d_close:.2f} m (cmd_vel zero)"
                )
                self._last_emergency_warn = now
            self._pub.publish(twist)
            return
        self._last_emergency_warn = None

        gap = self._best_gap_center(scan, fov)
        if gap is None:
            self._pub.publish(twist)
            return

        center_angle, _width = gap
        theta = _wrap_pi(center_angle)

        gain = float(self.get_parameter("angular_gain").value)
        w = gain * theta
        wmax = float(self.get_parameter("max_angular_z").value)
        w = max(-wmax, min(wmax, w))

        v = float(self.get_parameter("linear_x").value)
        align_thr = float(self.get_parameter("align_threshold_rad").value)
        scale = float(self.get_parameter("linear_scale_when_misaligned").value)
        if abs(theta) > align_thr:
            v *= scale

        twist.linear.x = v
        twist.angular.z = w
        if bool(self.get_parameter("invert_drive").value):
            twist.linear.x = -twist.linear.x
            twist.angular.z = -twist.angular.z
        self._pub.publish(twist)


def main() -> None:
    rclpy.init()
    node = GapNav()
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
