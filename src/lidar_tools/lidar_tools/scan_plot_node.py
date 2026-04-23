#!/usr/bin/env python3
"""
Live LiDAR plot: top-down map only. You at (0,0), environment in real X,Y (meters).
Full 360° around the LiDAR — everything as a circle/map.
"""

import math
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan

# Allow -p foo:=120 without INTEGER vs DOUBLE clashes from the ROS CLI.
_PARAM_NUM = ParameterDescriptor(dynamic_typing=True)


def _wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def _draw_distance_rings(ax, r_max, step=1.0):
    """Draw concentric circles at 1m, 2m, ..."""
    import numpy as np
    for r in np.arange(step, r_max + 0.01, step):
        theta = np.linspace(0, 2 * math.pi, 100)
        ax.plot(r * np.cos(theta), r * np.sin(theta), "k-", alpha=0.2, linewidth=0.8)
    for r in [1, 2, 3, 4, 5]:
        if r <= r_max:
            ax.text(r + 0.08, 0, f"{r}m", fontsize=8, alpha=0.7)


def _draw_visible_area_border(ax, fov_degrees, fov_center_rad, radius_m):
    """Draw the currently configured visible area boundary."""
    if radius_m <= 0:
        return

    import numpy as np

    border_color = "darkorange"
    border_lw = 2.0
    border_alpha = 0.95

    # Full view: draw one circle border.
    if fov_degrees >= 360.0:
        theta = np.linspace(0, 2 * math.pi, 300)
        ax.plot(
            radius_m * np.cos(theta),
            radius_m * np.sin(theta),
            color=border_color,
            linewidth=border_lw,
            alpha=border_alpha,
            linestyle="--",
            zorder=6,
        )
        return

    half = math.radians(fov_degrees / 2.0)
    a0 = fov_center_rad - half
    a1 = fov_center_rad + half

    # Two radial borders.
    ax.plot(
        [0, radius_m * math.cos(a0)],
        [0, radius_m * math.sin(a0)],
        color=border_color,
        linewidth=border_lw,
        alpha=border_alpha,
        zorder=6,
    )
    ax.plot(
        [0, radius_m * math.cos(a1)],
        [0, radius_m * math.sin(a1)],
        color=border_color,
        linewidth=border_lw,
        alpha=border_alpha,
        zorder=6,
    )

    # Outer arc border between FOV edges.
    arc = np.linspace(a0, a1, 220)
    ax.plot(
        radius_m * np.cos(arc),
        radius_m * np.sin(arc),
        color=border_color,
        linewidth=border_lw,
        alpha=border_alpha,
        linestyle="--",
        zorder=6,
    )


def _draw_visible_area_fill(ax, fov_degrees, fov_center_rad, radius_m):
    """Draw a light fill for the full sensor coverage area."""
    if radius_m <= 0:
        return

    import numpy as np

    if fov_degrees >= 360.0:
        theta = np.linspace(0, 2 * math.pi, 360)
        xs = radius_m * np.cos(theta)
        ys = radius_m * np.sin(theta)
        ax.fill(xs, ys, color="orange", alpha=0.08, zorder=1)
        return

    half = math.radians(fov_degrees / 2.0)
    a0 = fov_center_rad - half
    a1 = fov_center_rad + half
    arc = np.linspace(a0, a1, 260)
    xs = [0.0] + list(radius_m * np.cos(arc)) + [0.0]
    ys = [0.0] + list(radius_m * np.sin(arc)) + [0.0]
    ax.fill(xs, ys, color="orange", alpha=0.08, zorder=1)


class ScanPlotNode(Node):
    def __init__(self):
        super().__init__("scan_plot_node")

        self.declare_parameter("fov_degrees", 360.0, _PARAM_NUM)
        # Center of FOV cone in LaserScan angle frame (deg). 0 = forward = angle 0 in /scan.
        self.declare_parameter("fov_center_deg", 0.0, _PARAM_NUM)
        self.declare_parameter("max_range_m", 5.0, _PARAM_NUM)
        self.declare_parameter("max_scan_range_m", 0.0, _PARAM_NUM)
        self.declare_parameter("show_full_scan_area", True)
        fov = float(self.get_parameter("fov_degrees").value)
        self.fov_degrees = fov if fov > 0 else 360.0
        self.half_fov_rad = math.radians(self.fov_degrees / 2.0)
        self.fov_center_rad = math.radians(float(self.get_parameter("fov_center_deg").value))
        self.max_range_m = float(self.get_parameter("max_range_m").value)
        self.max_scan_range_m = float(self.get_parameter("max_scan_range_m").value)
        self.show_full_scan_area = bool(self.get_parameter("show_full_scan_area").value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.latest_scan = None
        self.sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_callback,
            qos,
        )

        range_info = f" (scan range limit: {self.max_scan_range_m}m)" if self.max_scan_range_m > 0 else ""
        self.get_logger().info(
            f"Subscribed to /scan. Top-down map (X,Y in meters).{range_info} Starting..."
        )
        self.setup_plot()

    def setup_plot(self):
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        self.plt = plt
        self.fig, self.ax = plt.subplots(figsize=(9, 9))
        self.plt.ion()
        self.plt.show()
        self._draw_idle_frame("Waiting for /scan data...")

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.redraw()

    def redraw(self):
        if self.latest_scan is None:
            self._draw_idle_frame("Waiting for /scan data...")
            return

        msg = self.latest_scan

        xs = []
        ys = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            if self.fov_degrees < 360.0:
                rel = _wrap_pi(angle - self.fov_center_rad)
                if abs(rel) > self.half_fov_rad:
                    continue
            # Optional: only show points within max_scan_range_m (e.g. 1.0 = 1 m only)
            if self.max_scan_range_m > 0 and float(r) > self.max_scan_range_m:
                continue
            x = float(r) * math.cos(angle)
            y = float(r) * math.sin(angle)
            xs.append(x)
            ys.append(y)

        self.ax.clear()

        # Sensor coverage radius: use configured scan cap if present, else scan message range_max.
        sensor_radius = (
            self.max_scan_range_m if self.max_scan_range_m > 0 else float(msg.range_max)
        )
        data_max = max(math.hypot(x, y) for x, y in zip(xs, ys)) if xs else 2.0
        if self.show_full_scan_area and sensor_radius > 0:
            lim = max(0.5, min(sensor_radius * 1.15, self.max_range_m))
        else:
            lim = max(0.5, min(data_max * 1.15, self.max_range_m))
        visible_radius = sensor_radius if sensor_radius > 0 else lim
        _draw_visible_area_fill(
            self.ax,
            self.fov_degrees,
            self.fov_center_rad,
            visible_radius,
        )
        _draw_visible_area_border(
            self.ax,
            self.fov_degrees,
            self.fov_center_rad,
            visible_radius,
        )

        if not xs:
            self.ax.set_title("Top-down map (X, Y in m) — no valid points")
            self.ax.set_xlabel("X (m)")
            self.ax.set_ylabel("Y (m)")
            self.ax.set_xlim(-lim, lim)
            self.ax.set_ylim(-lim, lim)
            self.ax.set_aspect("equal", adjustable="box")
            self.ax.grid(True, alpha=0.4)
            self.ax.axhline(0, color="gray", linewidth=0.5, alpha=0.6)
            self.ax.axvline(0, color="gray", linewidth=0.5, alpha=0.6)
            self.plt.draw()
            self.plt.pause(0.01)
            return

        # All points at their real (X, Y) position in meters
        self.ax.scatter(xs, ys, s=6, c="steelblue", alpha=0.85, edgecolors="none")
        # LiDAR at origin
        self.ax.scatter([0], [0], s=200, marker="^", c="red", edgecolors="black", linewidths=2, zorder=10)
        self.ax.plot(0, 0, "k+", markersize=12, markeredgewidth=2, zorder=10)
        # Distance rings (1m, 2m, ...)
        _draw_distance_rings(self.ax, lim)
        # Forward direction
        self.ax.arrow(0, 0, 0.35, 0, head_width=0.06, head_length=0.04, fc="red", ec="red", zorder=9)
        self.ax.text(0.42, 0.02, "X (forward)", fontsize=10, fontweight="bold")
        self.ax.text(0.02, 0.42, "Y (left)", fontsize=9, alpha=0.9)

        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("X (m)", fontsize=11)
        self.ax.set_ylabel("Y (m)", fontsize=11)
        self.ax.set_title("Top-down map — LiDAR at (0, 0), obstacles at real X,Y coordinates", fontsize=11)
        self.ax.grid(True, alpha=0.4)
        self.ax.axhline(0, color="gray", linewidth=0.5, alpha=0.6)
        self.ax.axvline(0, color="gray", linewidth=0.5, alpha=0.6)

        self.fig.tight_layout()
        self.plt.draw()
        self.plt.pause(0.01)

    def _draw_idle_frame(self, title):
        """Always draw a visible reference frame even without scan points."""
        self.ax.clear()
        radius = self.max_scan_range_m if self.max_scan_range_m > 0 else 2.0
        lim = max(0.5, min(radius * 1.15, self.max_range_m))

        _draw_visible_area_fill(
            self.ax,
            self.fov_degrees,
            self.fov_center_rad,
            radius,
        )
        _draw_visible_area_border(
            self.ax,
            self.fov_degrees,
            self.fov_center_rad,
            radius,
        )
        _draw_distance_rings(self.ax, lim)

        # LiDAR center and forward marker.
        self.ax.scatter([0], [0], s=200, marker="^", c="red", edgecolors="black", linewidths=2, zorder=10)
        self.ax.plot(0, 0, "k+", markersize=12, markeredgewidth=2, zorder=10)
        self.ax.arrow(0, 0, 0.35, 0, head_width=0.06, head_length=0.04, fc="red", ec="red", zorder=9)

        self.ax.set_xlim(-lim, lim)
        self.ax.set_ylim(-lim, lim)
        self.ax.set_aspect("equal", adjustable="box")
        self.ax.set_xlabel("X (m)", fontsize=11)
        self.ax.set_ylabel("Y (m)", fontsize=11)
        self.ax.set_title(title, fontsize=11)
        self.ax.grid(True, alpha=0.4)
        self.ax.axhline(0, color="gray", linewidth=0.5, alpha=0.6)
        self.ax.axvline(0, color="gray", linewidth=0.5, alpha=0.6)

        self.fig.tight_layout()
        self.plt.draw()
        self.plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPlotNode()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.plt.get_fignums():
                break
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
