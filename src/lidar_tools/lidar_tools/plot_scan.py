#!/usr/bin/env python3
"""
Standalone script to plot LiDAR scan from a CSV of (angle_deg, range) or (timestamp, range).
Can also be used to export one scan to CSV from ROS2 (run with --export and a bag/node).

Usage:
  # Plot from CSV (columns: angle_deg, range_m) or (stamp_sec, range_m)
  python3 plot_scan.py scan_data.csv

  # Plot from CSV with custom column indices (0=angle_deg, 1=range_m)
  python3 plot_scan.py scan_data.csv --angle-col 0 --range-col 1
"""

import argparse
import math
import sys
from pathlib import Path


def load_scan_csv(path, angle_col=0, range_col=1, skip_header=0):
    """Load angles (deg) and ranges (m) from CSV. Returns (angles_rad, ranges)."""
    angles_deg = []
    ranges = []
    with open(path) as f:
        for i, line in enumerate(f):
            if i < skip_header:
                continue
            parts = [p.strip() for p in line.split(",")]
            if len(parts) <= max(angle_col, range_col):
                continue
            try:
                a = float(parts[angle_col])
                r = float(parts[range_col])
                angles_deg.append(a)
                ranges.append(r)
            except ValueError:
                continue
    angles_rad = [math.radians(a) for a in angles_deg]
    return angles_rad, ranges


def plot_polar(angles_rad, ranges, out_path=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("Install matplotlib and numpy: pip3 install matplotlib numpy", file=sys.stderr)
        sys.exit(1)

    x = np.array(ranges) * np.cos(np.array(angles_rad))
    y = np.array(ranges) * np.sin(np.array(angles_rad))

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.scatter(x, y, s=2, c="blue", alpha=0.7)
    r_max = max(ranges) if ranges else 10.0
    ax.set_xlim(-r_max, r_max)
    ax.set_ylim(-r_max, r_max)
    ax.set_aspect("equal")
    ax.set_title("LiDAR scan (polar → Cartesian)")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True)
    plt.tight_layout()
    if out_path:
        plt.savefig(out_path, dpi=150)
        print(f"Saved plot to {out_path}")
    else:
        plt.show()


def plot_time_series(angles_rad, ranges, out_path=None):
    """Plot range vs angle (or index) as time-series style."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("Install matplotlib and numpy: pip3 install matplotlib numpy", file=sys.stderr)
        sys.exit(1)

    indices = list(range(len(ranges)))
    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(indices, ranges, "b.-", markersize=1)
    ax.set_xlabel("Sample index (angle)")
    ax.set_ylabel("Range (m)")
    ax.set_title("Range vs angle index")
    ax.grid(True)
    plt.tight_layout()
    if out_path:
        plt.savefig(out_path, dpi=150)
        print(f"Saved plot to {out_path}")
    else:
        plt.show()


def main():
    parser = argparse.ArgumentParser(description="Plot LiDAR scan from CSV")
    parser.add_argument("csv_file", nargs="?", help="CSV with angle_deg, range_m columns")
    parser.add_argument("--angle-col", type=int, default=0, help="Column index for angle (deg)")
    parser.add_argument("--range-col", type=int, default=1, help="Column index for range (m)")
    parser.add_argument("--skip-header", type=int, default=0, help="Skip N header lines")
    parser.add_argument("--polar", action="store_true", default=True, help="Plot polar (default)")
    parser.add_argument("--time-series", action="store_true", help="Plot range vs index")
    parser.add_argument("-o", "--output", help="Save figure to file instead of showing")
    args = parser.parse_args()

    if not args.csv_file or not Path(args.csv_file).exists():
        print("Usage: python3 plot_scan.py <scan_data.csv> [--output plot.png]", file=sys.stderr)
        print("CSV format: angle_deg, range_m (or use --angle-col / --range-col)", file=sys.stderr)
        sys.exit(1)

    angles_rad, ranges = load_scan_csv(
        args.csv_file,
        angle_col=args.angle_col,
        range_col=args.range_col,
        skip_header=args.skip_header,
    )
    if not angles_rad:
        print("No valid data in CSV.", file=sys.stderr)
        sys.exit(1)

    if args.polar:
        plot_polar(angles_rad, ranges, args.output)
    if args.time_series:
        out_ts = args.output.replace(".png", "_ts.png") if args.output else None
        plot_time_series(angles_rad, ranges, out_ts)


if __name__ == "__main__":
    main()
