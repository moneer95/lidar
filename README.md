# YDLidar G4 on Jetson Orin Nano with ROS2

This project lets you read YDLidar G4 data over ROS2 on a Jetson Orin Nano Developer Kit, capture timestamps, and plot the scan data.

## Prerequisites

- **Hardware:** Jetson Orin Nano Developer Kit, YDLidar G4 (USB)
- **OS:** Ubuntu 22.04 (JetPack 5.x or 6.x)
- **ROS2:** Humble (recommended for Orin Nano)

## Install ROS2 Humble on the Jetson (detailed)

**Follow the step-by-step guide:** [**docs/INSTALL_ROS2_JETSON.md**](docs/INSTALL_ROS2_JETSON.md)

That guide covers:

1. System update and locale (UTF-8)
2. Adding the official ROS2 apt repository
3. Installing `ros-humble-desktop` (or `ros-humble-ros-base`) and `ros-dev-tools`
4. Sourcing the environment and making it permanent
5. Verifying with talker/listener
6. Creating a workspace with colcon
7. Optional: YDLidar SDK, ydlidar_ros2_driver, and USB udev rules

After you finish that, come back here for the lidar_tools workspace and running the G4.

## 1. Install YDLidar SDK

```bash
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
make
sudo make install
```

## 2. Install YDLidar ROS2 driver (Humble)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 3. Configure for YDLidar G4

Create or edit the driver's config for G4. Example params (adjust port if needed):

- **Port:** Check with `ls /dev/ttyUSB*` or `ls /dev/ttyACM*` when the G4 is connected.
- **Baudrate:** 230400 for G4
- **Lidar type:** G4 (often `lidar_type: 0` or `1` in config; check driver docs)
- **Scan frequency:** G4 supports **5–12 Hz**. Set `frequency: 12` (or 10, 11) in the driver params to increase scan rate. See `config/ydlidar_g4_params.yaml` for an example.

Add udev rule so you don't need sudo:

```bash
echo 'KERNEL=="ttyUSB*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ydlidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# Unplug and replug the LiDAR
```

## 4. Build this workspace (lidar reader + plots)

The package lives in `src/lidar_tools/`. You **must build** the workspace and **source the install** before `ros2 run lidar_tools ...` will work.

**First time (from the project root, e.g. `~/Desktop/lidar`):**

- Make sure you are **not** inside Trash (your path must not contain `Trash`). If the project is in Trash, copy it to e.g. `~/Desktop/lidar` or `~/lidar` first.
- Open a **new terminal** (so stale `COLCON_PREFIX_PATH` / `AMENT_PREFIX_PATH` are cleared), then:

```bash
cd ~/Desktop/lidar
# Optional: clean if a previous build failed or paths are wrong
rm -rf build install log
unset COLCON_PREFIX_PATH AMENT_PREFIX_PATH
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Or run the helper script: `chmod +x clean_and_build.sh && ./clean_and_build.sh`

**Every new terminal** (or add to `~/.bashrc`):

```bash
source /opt/ros/humble/setup.bash
source ~/Desktop/lidar/install/setup.bash
```

Then run:

```bash
ros2 run lidar_tools scan_plot_node
```

Optional (for live and CSV plotting):

```bash
pip3 install -r requirements.txt
```

## 5. Run the LiDAR driver

**Terminal 1 – start the G4 driver:**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch ydlidar_ros2_driver ydlidar_launch.py
```

(If the package uses a different launch file name, use the one provided in `ydlidar_ros2_driver`.)

The driver publishes `sensor_msgs/msg/LaserScan` on `/scan` with:
- `header.stamp` – timestamp of the scan
- `angle_min`, `angle_max`, `angle_increment`
- `range_min`, `range_max`
- `ranges[]` – distance per ray
- `intensities[]` – if supported by G4

## 6. Read data and timestamps (formatted for obstacle avoidance / path planning)

**Terminal 2 – run the reader node.** It prints **time** and **coordinates (x, y)** for each scan in a clear table:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lidar_tools scan_reader_node
```

Example output:
- **SCAN #** and **TIME** (stamp + ROS time)
- **POINTS** count and **FRAME**
- Table: **idx**, **x (m)**, **y (m)**, **range (m)**, **angle (deg)** — LiDAR at origin, X forward, Y left.

**Save data to CSV with the new format** (human-readable time + coordinates, one row per point):

```bash
ros2 run lidar_tools scan_reader_node --points-csv ~/lidar_points.csv
```

Columns: `time_human`, `stamp_sec`, `stamp_nanosec`, `scan_id`, `point_idx`, `x_m`, `y_m`, `range_m`, `angle_deg`.  
- **time_human**: readable timestamp (e.g. `2025-03-12 14:30:45.123456`).  
- **x_m**, **y_m**: coordinates in meters (LiDAR at origin). Use this file for obstacle avoidance and path planning.

Limit how many points are printed per scan (default 80):

```bash
ros2 run lidar_tools scan_reader_node --max-print 50
```

Log scan metadata (timestamps, frame_id, angles, num_ranges) to CSV:

```bash
ros2 run lidar_tools scan_reader_node --csv
ros2 run lidar_tools scan_reader_node --csv --csv-path ~/my_scans.csv
```

Export one full scan (angle_deg, range_m) to CSV for graphing:

```bash
ros2 run lidar_tools export_scan_node --output scan_export.csv
```

Optional: record raw topics for later analysis:

```bash
ros2 bag record /scan
```

## 7. Plot the data

**Terminal 3 – live polar plot and time-series:**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lidar_tools scan_plot_node
```

**Show only 100° (centered on front):**

```bash
ros2 run lidar_tools scan_plot_node --ros-args -p fov_degrees:=100
```

**Limit scan to 1 m only** (only show points within 1 m; good for close-up view):

```bash
ros2 run lidar_tools scan_plot_node --ros-args -p max_scan_range_m:=1.0
```

**YDLidar G4 range:** min **0.12 m**, max **16 m**. Scan frequency 5–12 Hz.

Install matplotlib/numpy if needed: `pip3 install matplotlib numpy`

**Plot from an exported CSV (angle_deg, range_m):**

```bash
cd /path/to/lidar
python3 src/lidar_tools/lidar_tools/plot_scan.py scan_export.csv
python3 src/lidar_tools/lidar_tools/plot_scan.py scan_export.csv --fov 100 -o plot.png
```

## Project layout

- `src/lidar_tools/` – ROS2 package:
  - `scan_reader_node` – subscribes to `/scan`, prints formatted time + (x,y) table; optional `--points-csv` for obstacle/path data, `--csv` for metadata
  - `export_scan_node` – writes one scan to CSV (angle_deg, range_m) then exits
  - `scan_plot_node` – live polar + time-series graphs from `/scan`
  - `plot_scan.py` – standalone script to plot from CSV
- `README.md` – this file

## Troubleshooting

- **No `/scan`:** Check port and baudrate; ensure the driver launch uses the G4 config.
- **Permission denied:** Use the udev rule above and replug the USB.
- **Wrong angles/ranges:** Adjust `angle_min`, `angle_max`, `range_max` in the driver’s YAML for G4.
