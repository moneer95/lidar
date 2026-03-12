#!/usr/bin/env python3
"""
ROS2 node: subscribe to /scan and draw live graphs (polar plot + optional time series).
Use with YDLidar G4 driver publishing on /scan.
Requires: matplotlib, numpy (pip install matplotlib numpy)
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanPlotNode(Node):
    def __init__(self, history_len=50):
        super().__init__("scan_plot_node")
        self.history_len = history_len
        self._stamps = deque(maxlen=history_len)
        self._num_points = deque(maxlen=history_len)
        self._latest_scan = None

        self.sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            10,
        )
        self.get_logger().info("Subscribed to /scan. Starting plot (close window to stop).")

        self._setup_plot()

    def _setup_plot(self):
        try:
            import matplotlib
            matplotlib.use("TkAgg")
            import matplotlib.pyplot as plt
            import numpy as np
        except ImportError as e:
            self.get_logger().error(
                "matplotlib/numpy required for plotting. Install: pip3 install matplotlib numpy"
            )
            raise

        self.plt = plt
        self.np = np
        self.fig, (self.ax_polar, self.ax_time) = self.plt.subplots(1, 2, figsize=(12, 5))
        self.plt.ion()
        self.plt.show()

    def scan_callback(self, msg: LaserScan):
        stamp = msg.header.stamp
        t_sec = float(stamp.sec) + 1e-9 * float(stamp.nanosec)
        self._stamps.append(t_sec)
        self._num_points.append(len(msg.ranges))
        self._latest_scan = msg
        self._redraw()

    def _redraw(self):
        if self._latest_scan is None:
            return
        try:
            import numpy as np
        except ImportError:
            return

        msg = self._latest_scan
        self.ax_polar.clear()
        self.ax_time.clear()

        # Polar: angles and ranges (replace inf/nan with range_max for display)
        angles = []
        ranges = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if math.isinf(r) or (r != r):
                r = msg.range_max
            else:
                r = max(msg.range_min, min(float(r), msg.range_max))
            angles.append(angle)
            ranges.append(r)

        if angles and ranges:
            angles_np = np.array(angles)
            ranges_np = np.array(ranges)
            x = ranges_np * np.cos(angles_np)
            y = ranges_np * np.sin(angles_np)
            self.ax_polar.scatter(x, y, s=1, c="blue", alpha=0.6)
        self.ax_polar.set_xlim(-msg.range_max, msg.range_max)
        self.ax_polar.set_ylim(-msg.range_max, msg.range_max)
        self.ax_polar.set_aspect("equal")
        self.ax_polar.set_title("LaserScan (polar → Cartesian)")
        self.ax_polar.set_xlabel("x (m)")
        self.ax_polar.set_ylabel("y (m)")
        self.ax_polar.grid(True)

        # Time series: stamp vs number of points (or use mean range per scan if you prefer)
        if len(self._stamps) > 1:
            stamps = list(self._stamps)
            self.ax_time.plot(stamps, list(self._num_points), "b.-")
            self.ax_time.set_xlabel("Timestamp (sec)")
            self.ax_time.set_ylabel("Number of range points")
            self.ax_time.set_title("Scan size over time")
        else:
            self.ax_time.set_title("Scan size over time (waiting for data)")
            self.ax_time.set_ylabel("Number of range points")

        self.ax_time.grid(True)
        self.plt.draw()
        self.plt.pause(0.02)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPlotNode(history_len=100)
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
