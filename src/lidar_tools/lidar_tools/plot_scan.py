#!/usr/bin/env python3
"""
Plot LiDAR scan from CSV (angle_deg, range_m) with clear views:
  - Polar: angle (degrees) vs range (m) — "how far at each direction"
  - Top-down: x,y with distance rings and labels
"""

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
    return angles_rad, ranges, angles_deg


def _draw_distance_rings(ax, r_max, step=1.0):
    import numpy as np
    for r in np.arange(step, r_max + 0.01, step):
        theta = np.linspace(0, 2 * math.pi, 100)
        ax.plot(r * np.cos(theta), r * np.sin(theta), "k-", alpha=0.25, linewidth=0.8)
    if r_max >= 1.0:
        ax.text(1.05, 0, "1m", fontsize=8, alpha=0.7)
    if r_max >= 2.0:
        ax.text(2.05, 0, "2m", fontsize=8, alpha=0.7)


def _filter_fov(angles_rad, ranges, angles_deg, fov_degrees):
    """Keep only points within ±fov_degrees/2 around 0°."""
    if fov_degrees >= 360:
        return angles_rad, ranges, angles_deg
    half = fov_degrees / 2.0
    out_rad, out_r, out_deg = [], [], []
    for a_rad, r, a_deg in zip(angles_rad, ranges, angles_deg):
        if -half <= a_deg <= half:
            out_rad.append(a_rad)
            out_r.append(r)
            out_deg.append(a_deg)
    return out_rad, out_r, out_deg


def plot_both(angles_rad, ranges, angles_deg, out_path=None, fov_degrees=360):
    """Polar + top-down in one figure with stats and labels."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import numpy as np
    except ImportError:
        print("Install matplotlib and numpy: pip3 install matplotlib numpy", file=sys.stderr)
        sys.exit(1)

    angles_rad, ranges, angles_deg = _filter_fov(angles_rad, ranges, angles_deg, fov_degrees)
    if not ranges:
        print("No points in selected FOV.", file=sys.stderr)
        sys.exit(1)

    r_min, r_max = min(ranges), max(ranges)
    r_mean = sum(ranges) / len(ranges)
    lim_top = max(2.0, min(r_max * 1.1, 15.0))

    fig, (ax_polar, ax_top) = plt.subplots(1, 2, figsize=(14, 6))

    # ---- Polar: angle (deg) vs range (m) ----
    ax_polar.scatter(angles_deg, ranges, s=4, c=ranges, cmap="viridis", alpha=0.9)
    ax_polar.set_xlabel("Angle (degrees)\n0° = front, ±180° = back")
    ax_polar.set_ylabel("Range (m)")
    ax_polar.set_title(f"Polar: distance at each angle (FOV {fov_degrees}°)")
    if fov_degrees < 360:
        ax_polar.set_xlim(-fov_degrees / 2, fov_degrees / 2)
    ax_polar.set_ylim(0, lim_top)
    ax_polar.axhline(y=1, color="gray", linestyle="--", alpha=0.5)
    ax_polar.axhline(y=2, color="gray", linestyle="--", alpha=0.5)
    ax_polar.grid(True, alpha=0.5)
    stats = f"Points: {len(ranges)}\nMin: {r_min:.2f} m\nMax: {r_max:.2f} m\nMean: {r_mean:.2f} m"
    ax_polar.text(0.02, 0.98, stats, transform=ax_polar.transAxes,
                  fontsize=9, verticalalignment="top", family="monospace",
                  bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))

    # ---- Top-down: x,y with rings ----
    xs = [r * math.cos(a) for a, r in zip(angles_rad, ranges)]
    ys = [r * math.sin(a) for a, r in zip(angles_rad, ranges)]
    ax_top.scatter(xs, ys, s=4, c=ranges, cmap="viridis", alpha=0.9)
    ax_top.scatter([0], [0], s=150, marker="^", c="red", edgecolors="black", linewidths=1.5, zorder=5, label="LiDAR")
    _draw_distance_rings(ax_top, lim_top)
    ax_top.arrow(0, 0, 0.4, 0, head_width=0.08, head_length=0.06, fc="red", ec="red")
    ax_top.text(0.5, 0.05, "Front (0°)", fontsize=9, fontweight="bold")
    ax_top.text(-0.6, 0.05, "Back", fontsize=8, alpha=0.8)
    ax_top.set_xlim(-lim_top, lim_top)
    ax_top.set_ylim(-lim_top, lim_top)
    ax_top.set_aspect("equal", adjustable="box")
    ax_top.set_xlabel("X (m) — forward")
    ax_top.set_ylabel("Y (m) — left")
    ax_top.set_title(f"Top-down (FOV {fov_degrees}°): you are at the red triangle")
    ax_top.grid(True, alpha=0.4)

    plt.tight_layout()
    if out_path:
        plt.savefig(out_path, dpi=150)
        print(f"Saved to {out_path}")
    else:
        plt.show()


def plot_polar_only(angles_rad, ranges, angles_deg, out_path=None, fov_degrees=360):
    """Single polar plot (angle vs range)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Install matplotlib and numpy: pip3 install matplotlib numpy", file=sys.stderr)
        sys.exit(1)

    angles_rad, ranges, angles_deg = _filter_fov(angles_rad, ranges, angles_deg, fov_degrees)
    if not ranges:
        print("No points in selected FOV.", file=sys.stderr)
        sys.exit(1)
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.scatter(angles_deg, ranges, s=4, c=ranges, cmap="viridis", alpha=0.9)
    ax.set_xlabel("Angle (degrees) — 0° = front")
    ax.set_ylabel("Range (m)")
    ax.set_title(f"Polar: distance at each angle (FOV {fov_degrees}°)")
    if fov_degrees < 360:
        ax.set_xlim(-fov_degrees / 2, fov_degrees / 2)
    r_max = max(ranges) if ranges else 10.0
    ax.set_ylim(0, r_max * 1.05)
    ax.grid(True, alpha=0.5)
    plt.tight_layout()
    if out_path:
        plt.savefig(out_path, dpi=150)
        print(f"Saved to {out_path}")
    else:
        plt.show()


def main():
    import argparse
    parser = argparse.ArgumentParser(
        description="Plot LiDAR scan from CSV (angle_deg, range_m). Two panels: polar + top-down.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python3 plot_scan.py scan_export.csv
  python3 plot_scan.py scan_export.csv --fov 100
  python3 plot_scan.py scan_export.csv -o my_scan.png --fov 100
  python3 plot_scan.py scan_export.csv --polar-only
"""
    )
    parser.add_argument("csv_file", nargs="?", help="CSV with angle_deg, range_m")
    parser.add_argument("--angle-col", type=int, default=0)
    parser.add_argument("--range-col", type=int, default=1)
    parser.add_argument("--skip-header", type=int, default=0)
    parser.add_argument("--polar-only", action="store_true", help="Only plot angle vs range (one panel)")
    parser.add_argument("--fov", type=float, default=360, metavar="DEG",
                        help="Show only this many degrees centered on 0° (e.g. 100 for ±50°). Default: 360")
    parser.add_argument("-o", "--output", help="Save figure to file")
    args = parser.parse_args()

    if not args.csv_file or not Path(args.csv_file).exists():
        print("Usage: python3 plot_scan.py <scan_export.csv> [-o plot.png]", file=sys.stderr)
        print("CSV format: angle_deg, range_m (one row per ray)", file=sys.stderr)
        sys.exit(1)

    angles_rad, ranges, angles_deg = load_scan_csv(
        args.csv_file,
        angle_col=args.angle_col,
        range_col=args.range_col,
        skip_header=args.skip_header,
    )
    if not angles_rad:
        print("No valid data in CSV.", file=sys.stderr)
        sys.exit(1)

    fov = 360 if args.fov <= 0 else min(360, args.fov)
    if args.polar_only:
        plot_polar_only(angles_rad, ranges, angles_deg, args.output, fov)
    else:
        plot_both(angles_rad, ranges, angles_deg, args.output, fov)


if __name__ == "__main__":
    main()
