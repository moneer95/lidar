#!/usr/bin/env python3
"""
Live LiDAR plot: polar view (angle vs range) + top-down view with distance rings.
Clear labels and stats so you can understand the scan at a glance.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


def _draw_distance_rings(ax, r_max, step=1.0):
    """Draw concentric circles at 1m, 2m, ... and add labels."""
    import numpy as np
    for r in np.arange(step, r_max + 0.01, step):
        theta = np.linspace(0, 2 * math.pi, 100)
        ax.plot(r * np.cos(theta), r * np.sin(theta), "k-", alpha=0.25, linewidth=0.8)
    # Label one ring
    if r_max >= 1.0:
        ax.text(1.05, 0, "1m", fontsize=8, alpha=0.7)
    if r_max >= 2.0:
        ax.text(2.05, 0, "2m", fontsize=8, alpha=0.7)


class ScanPlotNode(Node):
    def __init__(self):
        super().__init__("scan_plot_node")

        # Only show this many degrees (centered on front 0°). 0 = show full 360°
        self.declare_parameter("fov_degrees", 360)
        fov = self.get_parameter("fov_degrees").get_value_as_int()
        self.fov_degrees = fov if fov > 0 else 360
        self.half_fov_rad = math.radians(self.fov_degrees / 2.0)

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

        self.get_logger().info(
            f"Subscribed to /scan. FOV: {self.fov_degrees}° (0° = front). Starting live plot..."
        )
        self.setup_plot()

    def setup_plot(self):
        import matplotlib
        matplotlib.use("TkAgg")
        import matplotlib.pyplot as plt
        self.plt = plt
        self.fig, (self.ax_polar, self.ax_top) = plt.subplots(1, 2, figsize=(14, 6))
        self.plt.ion()
        self.plt.show()

    def scan_callback(self, msg):
        self.latest_scan = msg
        self.redraw()

    def redraw(self):
        if self.latest_scan is None:
            return

        import numpy as np
        msg = self.latest_scan

        angles_rad = []
        ranges = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            # Optional: only show points inside FOV (symmetric around 0°)
            if self.fov_degrees < 360 and abs(angle) > self.half_fov_rad:
                continue
            angles_rad.append(angle)
            ranges.append(float(r))

        if not ranges:
            self.ax_polar.clear()
            self.ax_top.clear()
            self.ax_polar.set_title("Polar: Angle vs Range (no valid points)")
            self.ax_top.set_title("Top-down view (no valid points)")
            self.plt.draw()
            self.plt.pause(0.01)
            return

        angles_deg = [math.degrees(a) for a in angles_rad]
        r_min, r_max = min(ranges), max(ranges)
        r_mean = sum(ranges) / len(ranges)
        lim_top = max(2.0, min(msg.range_max, 8.0))

        # ---- Left: Polar (Angle vs Range) - "radar" style ----
        self.ax_polar.clear()
        self.ax_polar.scatter(angles_deg, ranges, s=3, c=ranges, cmap="viridis", alpha=0.9)
        self.ax_polar.set_xlabel("Angle (degrees)\n0° = front, ±180° = back")
        self.ax_polar.set_ylabel("Range (m)")
        self.ax_polar.set_title(f"Polar: distance at each angle (FOV {self.fov_degrees}°)")
        if self.fov_degrees < 360:
            self.ax_polar.set_xlim(-self.fov_degrees / 2, self.fov_degrees / 2)
        self.ax_polar.set_ylim(0, lim_top)
        self.ax_polar.axhline(y=1, color="gray", linestyle="--", alpha=0.5)
        self.ax_polar.axhline(y=2, color="gray", linestyle="--", alpha=0.5)
        self.ax_polar.grid(True, alpha=0.5)
        # Stats text
        stats = f"Points: {len(ranges)}\nMin: {r_min:.2f} m\nMax: {r_max:.2f} m\nMean: {r_mean:.2f} m"
        self.ax_polar.text(0.02, 0.98, stats, transform=self.ax_polar.transAxes,
                           fontsize=8, verticalalignment="top", family="monospace",
                           bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))

        # ---- Right: Top-down (X, Y) with rings and directions ----
        self.ax_top.clear()
        xs = [r * math.cos(a) for a, r in zip(angles_rad, ranges)]
        ys = [r * math.sin(a) for a, r in zip(angles_rad, ranges)]
        self.ax_top.scatter(xs, ys, s=4, c=ranges, cmap="viridis", alpha=0.9)
        # LiDAR at origin
        self.ax_top.scatter([0], [0], s=120, marker="^", c="red", edgecolors="black", linewidths=1.5, zorder=5, label="LiDAR")
        # Distance rings
        _draw_distance_rings(self.ax_top, lim_top)
        # Cardinal directions (ROS: X forward, Y left)
        self.ax_top.arrow(0, 0, 0.4, 0, head_width=0.08, head_length=0.06, fc="red", ec="red")
        self.ax_top.text(0.5, 0.05, "Front (0°)", fontsize=9, fontweight="bold")
        self.ax_top.text(-0.6, 0.05, "Back", fontsize=8, alpha=0.8)
        self.ax_top.text(0.05, 0.5, "Left", fontsize=8, alpha=0.8)
        self.ax_top.text(-0.35, 0.5, "Right", fontsize=8, alpha=0.8)
        self.ax_top.set_xlim(-lim_top, lim_top)
        self.ax_top.set_ylim(-lim_top, lim_top)
        self.ax_top.set_aspect("equal", adjustable="box")
        self.ax_top.set_xlabel("X (m) — forward")
        self.ax_top.set_ylabel("Y (m) — left")
        self.ax_top.set_title(f"Top-down (FOV {self.fov_degrees}°): you are at the red triangle")
        self.ax_top.grid(True, alpha=0.4)

        self.fig.tight_layout()
        self.plt.draw()
        self.plt.pause(0.01)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPlotNode()
    # Allow override from command line: ros2 run lidar_tools scan_plot_node --ros-args -p fov_degrees:=100

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if not node.plt.get_fignums():
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
