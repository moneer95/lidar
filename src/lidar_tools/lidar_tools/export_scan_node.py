#!/usr/bin/env python3
"""
ROS2 node: wait for one /scan message, write (angle_deg, range_m) to CSV and exit.
Use output with: python3 plot_scan.py <output.csv>
"""

import math
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ExportScanNode(Node):
    def __init__(self, output_path):
        super().__init__("export_scan_node")
        self.output_path = output_path
        self.done = False
        self.sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            10,
        )
        self.get_logger().info(f"Waiting for one /scan message, will write to {output_path}")

    def scan_callback(self, msg: LaserScan):
        if self.done:
            return
        self.done = True
        stamp = msg.header.stamp
        t_sec = float(stamp.sec) + 1e-9 * float(stamp.nanosec)
        with open(self.output_path, "w") as f:
            f.write("angle_deg,range_m\n")
            for i, r in enumerate(msg.ranges):
                angle_rad = msg.angle_min + i * msg.angle_increment
                angle_deg = math.degrees(angle_rad)
                if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                    f.write(f"{angle_deg},{r}\n")
                else:
                    # use range_max for inf/nan so plot has a point
                    f.write(f"{angle_deg},{msg.range_max}\n")
        self.get_logger().info(
            f"Exported {len(msg.ranges)} points (stamp={t_sec:.3f}) to {self.output_path}"
        )


def main(args=None):
    rclpy.init(args=args)
    out = "scan_export.csv"
    if "--output" in (sys.argv or []):
        i = sys.argv.index("--output")
        if i + 1 < len(sys.argv):
            out = sys.argv[i + 1]
    node = ExportScanNode(out)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
