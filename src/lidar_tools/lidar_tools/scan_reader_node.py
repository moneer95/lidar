#!/usr/bin/env python3
"""
ROS2 node: subscribe to /scan (sensor_msgs/LaserScan).
Print formatted data: timestamp + (x, y) coordinates for obstacle avoidance and path planning.
"""

import csv
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan


class ScanReaderNode(Node):
    def __init__(self, save_csv=False, csv_path=None, print_points=False, points_csv_path=None, max_points_print=80):
        super().__init__("scan_reader_node")
        self.save_csv = save_csv
        self.csv_path = csv_path or str(Path.home() / "lidar_scans.csv")
        self.print_points = print_points
        self.points_csv_path = points_csv_path
        self.max_points_print = max(10, min(500, max_points_print))
        self.scan_count = 0
        self._csv_file = None
        self._csv_writer = None
        self._points_csv_file = None
        self._points_csv_writer = None

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.sub = self.create_subscription(
            LaserScan,
            "scan",
            self.scan_callback,
            qos,
        )

        self.get_logger().info(
            "Subscribed to /scan. Formatted output: time + (x, y) coordinates."
        )
        if self.save_csv:
            self._open_csv()
        if self.points_csv_path:
            self._open_points_csv()

    def _open_csv(self):
        self._csv_file = open(self.csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow([
            "scan_id", "stamp_sec", "stamp_nanosec", "stamp_ros_sec", "frame_id",
            "angle_min", "angle_max", "angle_increment", "range_min", "range_max",
            "num_ranges", "scan_time", "time_increment",
        ])
        self.get_logger().info(f"Writing scan metadata to {self.csv_path}")

    def _open_points_csv(self):
        self._points_csv_file = open(self.points_csv_path, "w", newline="")
        self._points_csv_writer = csv.writer(self._points_csv_file)
        self._points_csv_writer.writerow([
            "time_human", "stamp_sec", "stamp_nanosec", "scan_id", "point_idx", "x_m", "y_m", "range_m", "angle_deg",
        ])
        self.get_logger().info(f"Writing points (x,y) to {self.points_csv_path}")

    def _get_points(self, msg):
        """Return list of (x, y, range, angle_deg) for valid points."""
        points = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            x = float(r) * math.cos(angle)
            y = float(r) * math.sin(angle)
            angle_deg = math.degrees(angle)
            points.append((x, y, float(r), angle_deg))
        return points

    def scan_callback(self, msg: LaserScan):
        stamp = msg.header.stamp
        sec = stamp.sec
        nanosec = stamp.nanosec
        ros_time_sec = float(sec) + 1e-9 * float(nanosec)

        # Human-readable time (local timezone)
        try:
            dt = datetime.fromtimestamp(ros_time_sec)
            frac = f".{nanosec:09d}"[:7].rstrip("0") or ".0"
            human_time = dt.strftime("%H:%M:%S") + frac
            human_time_csv = dt.strftime("%Y-%m-%d %H:%M:%S") + frac
        except (OSError, ValueError):
            human_time = "(time out of range)"
            human_time_csv = ""

        self.scan_count += 1
        points = self._get_points(msg)

        # ----- Formatted console output: time + coordinates -----
        sep = "=" * 72
        print()
        print(sep)
        print(f"  SCAN #{self.scan_count}  |  TIME: {human_time}  (ROS: {ros_time_sec:.6f} s)")
        print(f"  FRAME: {msg.header.frame_id}  |  POINTS: {len(points)}  (valid)")
        print(sep)
        print(f"  {'idx':>5}  {'x (m)':>10}  {'y (m)':>10}  {'range (m)':>10}  {'angle (deg)':>11}")
        print("  " + "-" * 52)

        to_print = points[: self.max_points_print]
        for idx, (x, y, r, a) in enumerate(to_print):
            print(f"  {idx:>5}  {x:>10.4f}  {y:>10.4f}  {r:>10.4f}  {a:>11.2f}")
        if len(points) > self.max_points_print:
            print(f"  ... and {len(points) - self.max_points_print} more points (use --points-csv to get all)")
        print(sep)
        print()

        # ----- Metadata CSV (optional) -----
        if self.save_csv and self._csv_writer:
            self._csv_writer.writerow([
                self.scan_count, sec, nanosec, f"{ros_time_sec:.9f}", msg.header.frame_id,
                msg.angle_min, msg.angle_max, msg.angle_increment,
                msg.range_min, msg.range_max, len(msg.ranges), msg.scan_time, msg.time_increment,
            ])
            self._csv_file.flush()

        # ----- Points CSV: one row per point (with human-readable time) -----
        if self._points_csv_writer:
            for idx, (x, y, r, a) in enumerate(points):
                self._points_csv_writer.writerow([
                    human_time_csv, sec, nanosec, self.scan_count, idx, f"{x:.6f}", f"{y:.6f}", f"{r:.6f}", f"{a:.4f}",
                ])
            self._points_csv_file.flush()

    def destroy_node(self, *args, **kwargs):
        for f in (self._csv_file, self._points_csv_file):
            if f:
                try:
                    f.close()
                except Exception:
                    pass
        super().destroy_node(*args, **kwargs)


def main(args=None):
    argv = sys.argv or []
    save_csv = "--csv" in argv
    csv_path = None
    for i, arg in enumerate(argv):
        if arg == "--csv-path" and i + 1 < len(argv):
            csv_path = argv[i + 1]
            break

    print_points = "--print-points" in argv or "--format" in argv
    points_csv_path = None
    for i, arg in enumerate(argv):
        if arg in ("--points-csv", "--points-csv-path") and i + 1 < len(argv):
            points_csv_path = argv[i + 1]
            break

    max_pp = 80
    for i, arg in enumerate(argv):
        if arg == "--max-print" and i + 1 < len(argv):
            try:
                max_pp = int(argv[i + 1])
            except ValueError:
                pass
            break

    rclpy.init(args=args)
    node = ScanReaderNode(
        save_csv=save_csv,
        csv_path=csv_path,
        print_points=True,  # always print formatted time + (x,y) table
        points_csv_path=points_csv_path,
        max_points_print=max_pp,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
