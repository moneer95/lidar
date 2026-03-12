# YDLidar G4 on Jetson Orin Nano with ROS2

This project lets you read YDLidar G4 data over ROS2 on a Jetson Orin Nano Developer Kit, capture timestamps, and plot the scan data.

## Prerequisites

- **Hardware:** Jetson Orin Nano Developer Kit, YDLidar G4 (USB)
- **OS:** Ubuntu 22.04 (JetPack 5.x or 6.x)
- **ROS2:** Humble (recommended for Orin Nano)

## 1. Install ROS2 Humble (if not already)

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 repo and install
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
```

## 2. Install YDLidar SDK

```bash
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
make
sudo make install
```

## 3. Install YDLidar ROS2 driver (Humble)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

## 4. Configure for YDLidar G4

Create or edit the driver's config for G4. Example params (adjust port if needed):

- **Port:** Check with `ls /dev/ttyUSB*` or `ls /dev/ttyACM*` when the G4 is connected.
- **Baudrate:** 230400 for G4
- **Lidar type:** G4 (often `lidar_type: 0` or `1` in config; check driver docs)

Add udev rule so you don't need sudo:

```bash
echo 'KERNEL=="ttyUSB*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ydlidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
# Unplug and replug the LiDAR
```

## 5. Build this workspace (lidar reader + plots)

Copy this `lidar` folder to your Jetson (or clone there). The package is already under `src/lidar_tools/`.

```bash
cd ~/lidar   # or wherever you put this project
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Optional (for live and CSV plotting):

```bash
pip3 install -r requirements.txt
```

## 6. Run the LiDAR driver

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

## 7. Read data and timestamps

**Terminal 2 – run the reader node (logs timestamps and optional CSV):**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lidar_tools scan_reader_node
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

## 8. Plot the data

**Terminal 3 – live polar plot and time-series:**

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run lidar_tools scan_plot_node
```

Install matplotlib/numpy if needed: `pip3 install matplotlib numpy`

**Plot from an exported CSV (angle_deg, range_m):**

```bash
cd /path/to/lidar
python3 src/lidar_tools/lidar_tools/plot_scan.py scan_export.csv
python3 src/lidar_tools/lidar_tools/plot_scan.py scan_export.csv --time-series -o plot.png
```

## Project layout

- `src/lidar_tools/` – ROS2 package:
  - `scan_reader_node` – subscribes to `/scan`, prints timestamps, optional `--csv` for metadata CSV
  - `export_scan_node` – writes one scan to CSV (angle_deg, range_m) then exits
  - `scan_plot_node` – live polar + time-series graphs from `/scan`
  - `plot_scan.py` – standalone script to plot from CSV
- `README.md` – this file

## Troubleshooting

- **No `/scan`:** Check port and baudrate; ensure the driver launch uses the G4 config.
- **Permission denied:** Use the udev rule above and replug the USB.
- **Wrong angles/ranges:** Adjust `angle_min`, `angle_max`, `range_max` in the driver’s YAML for G4.
