#!/usr/bin/env python3
"""
ROS2 node: subscribe to /scan (sensor_msgs/LaserScan), log timestamps and optional CSV.
Use with YDLidar G4 driver publishing on /scan.
"""

import csv
import os
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanReaderNode(Node):
    def __init__(self, save_csv=False, csv_path=None):
        super().__init__("scan_reader_node")
        self.save_csv = save_csv
        self.csv_path = csv_path or str(Path.home() / "lidar_scans.csv")
        self.scan_count = 0
        self._csv_file = None
        self._csv_writer = None



        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

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
            f"Subscribed to /scan. Timestamps will be printed. Save CSV: {save_csv}"
        )
        if self.save_csv:
            self._open_csv()

    def _open_csv(self):
        self._csv_file = open(self.csv_path, "w", newline="")
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            [
                "scan_id",
                "stamp_sec",
                "stamp_nanosec",
                "stamp_ros_sec",
                "frame_id",
                "angle_min",
                "angle_max",
                "angle_increment",
                "range_min",
                "range_max",
                "num_ranges",
                "scan_time",
                "time_increment",
            ]
        )
        self.get_logger().info(f"Writing scan metadata to {self.csv_path}")

    def scan_callback(self, msg: LaserScan):
        stamp = msg.header.stamp
        sec = stamp.sec
        nanosec = stamp.nanosec
        ros_time_sec = float(sec) + 1e-9 * float(nanosec)

        self.scan_count += 1
        self.get_logger().info(
            f"[{self.scan_count}] stamp: {sec}.{nanosec:09d} (ros_sec={ros_time_sec:.6f}) "
            f"frame={msg.header.frame_id} ranges={len(msg.ranges)} "
            f"angle=[{msg.angle_min:.3f}, {msg.angle_max:.3f}]"
        )

        if self.save_csv and self._csv_writer:
            self._csv_writer.writerow(
                [
                    self.scan_count,
                    sec,
                    nanosec,
                    f"{ros_time_sec:.9f}",
                    msg.header.frame_id,
                    msg.angle_min,
                    msg.angle_max,
                    msg.angle_increment,
                    msg.range_min,
                    msg.range_max,
                    len(msg.ranges),
                    msg.scan_time,
                    msg.time_increment,
                ]
            )
            self._csv_file.flush()

    def destroy_node(self, *args, **kwargs):
        if self._csv_file:
            try:
                self._csv_file.close()
            except Exception:
                pass
        super().destroy_node(*args, **kwargs)


def main(args=None):
    rclpy.init(args=args)
    save_csv = "--csv" in (sys.argv or [])
    csv_path = None
    for i, arg in enumerate(sys.argv or []):
        if arg == "--csv-path" and i + 1 < len(sys.argv):
            csv_path = sys.argv[i + 1]
            break
    node = ScanReaderNode(save_csv=save_csv, csv_path=csv_path)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
