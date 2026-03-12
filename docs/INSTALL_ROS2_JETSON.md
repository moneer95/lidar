# Detailed: Install ROS2 Humble on Jetson Orin Nano

This guide walks you through installing ROS2 Humble on a **Jetson Orin Nano Developer Kit** running **Ubuntu 22.04** (JetPack 5.x or 6.x), so everything works for building and running ROS2 nodes (including the YDLidar G4 and this project).

---

## Prerequisites

- **Board:** Jetson Orin Nano Developer Kit  
- **OS:** Ubuntu 22.04 (Jammy). Check with:
  ```bash
  cat /etc/os-release
  # VERSION should be 22.04
  ```
- **Network:** Internet access (Wi‑Fi or Ethernet).
- **User:** You need `sudo` access.

If you have Ubuntu 20.04, ROS2 Humble is not officially supported via deb packages on 20.04; use 22.04 (e.g. JetPack 6 / newer L4T) or build from source.

---

## Step 1: Update the system

Open a terminal on the Jetson and run:

```bash
sudo apt update
sudo apt upgrade -y
```

Reboot if the kernel or critical packages were updated:

```bash
sudo reboot
```

After reboot, log back in and continue.

---

## Step 2: Set locale (required for ROS2)

ROS2 expects a UTF-8 locale. Run:

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

Verify:

```bash
locale
```

You should see `LANG=en_US.UTF-8` and `LC_ALL=en_US.UTF-8`.

---

## Step 3: Enable Ubuntu Universe and install helpers

```bash
sudo apt install -y software-properties-common curl
sudo add-apt-repository universe
sudo apt update
```

---

## Step 4: Add the ROS2 apt repository

ROS2 provides a package that configures the repository and keys. Run:

```bash
# Download and install the ROS2 apt source package (for Ubuntu 22.04 / Jammy)
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
```

If the GitHub call fails (e.g. no internet or rate limit), download the latest `ros2-apt-source_*_jammy_all.deb` from [ros-apt-source releases](https://github.com/ros-infrastructure/ros-apt-source/releases), then:

```bash
sudo dpkg -i /path/to/ros2-apt-source_*_jammy_all.deb
```

Then refresh the package list:

```bash
sudo apt update
```

---

## Step 5: Upgrade system (important on 22.04)

Before installing ROS2, upgrade so `systemd` and `udev` are up to date. This avoids known issues with ROS2 dependencies:

```bash
sudo apt upgrade -y
```

---

## Step 6: Install ROS2 Humble

Choose one of the following.

**Option A – Desktop (recommended if you have a display):**  
Includes RViz, demos, and GUI tools.

```bash
sudo apt install -y ros-humble-desktop
```

**Option B – ROS-Base (minimal, no GUI):**  
Use this for headless or resource-limited setups.

```bash
sudo apt install -y ros-humble-ros-base
```

**Install development tools** (needed to build your own packages with colcon):

```bash
sudo apt install -y ros-dev-tools
```

On some setups you may also want:

```bash
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

---

## Step 7: Set up the environment (sourcing)

Each time you open a new terminal, run:

```bash
source /opt/ros/humble/setup.bash
```

To make this automatic for your user, add it to `~/.bashrc`:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

If you use **zsh**:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.zshrc
source ~/.zshrc
```

---

## Step 8: Verify the installation

**Terminal 1 – start a talker:**

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

You should see lines like: `Publishing: 'Hello World: 1'`, `'Hello World: 2'`, etc.

**Terminal 2 – start a listener:**

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
```

You should see: `I heard: [Hello World: N]`.  
If both work, ROS2 is installed and running correctly. Stop both with `Ctrl+C`.

---

## Step 9: Build a workspace (for your own packages)

You need a workspace to build the YDLidar driver and this lidar project.

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

After building, always source the workspace in new terminals:

```bash
source ~/ros2_ws/install/setup.bash
```

You can add that to `~/.bashrc` **after** the ROS2 Humble source line:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc   # only after ros2_ws exists
source ~/.bashrc
```

---

## Step 10: Optional – Install YDLidar SDK and driver (for G4)

When you are ready to use the YDLidar G4:

**10.1 – Install YDLidar SDK**

```bash
cd ~
git clone https://github.com/YDLIDAR/YDLidar-SDK.git
cd YDLidar-SDK
mkdir build && cd build
cmake ..
make
sudo make install
```

**10.2 – Clone and build the ROS2 driver (Humble branch)**

```bash
cd ~/ros2_ws/src
git clone -b humble https://github.com/YDLIDAR/ydlidar_ros2_driver.git
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

**10.3 – USB permissions for the LiDAR**

```bash
echo 'KERNEL=="ttyUSB*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-ydlidar.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

Unplug and replug the YDLidar G4. Check the port:

```bash
ls /dev/ttyUSB*
# or
ls /dev/ttyACM*
```

Use that device (e.g. `/dev/ttyUSB0`) in the driver’s launch file or config.

---

## Quick reference – useful commands

| Task | Command |
|------|--------|
| Source ROS2 (current shell) | `source /opt/ros/humble/setup.bash` |
| Source workspace | `source ~/ros2_ws/install/setup.bash` |
| List topics | `ros2 topic list` |
| List nodes | `ros2 node list` |
| Echo a topic | `ros2 topic echo /topic_name` |
| Build workspace | `cd ~/ros2_ws && colcon build --symlink-install` |

---

## Troubleshooting

- **`ros2: command not found`**  
  Run `source /opt/ros/humble/setup.bash` (or add it to `.bashrc` and open a new terminal).

- **Permission denied on `/dev/ttyUSB0`**  
  Add the udev rule in Step 10.3, reload udev, and replug the device. Alternatively: `sudo chmod 666 /dev/ttyUSB0` (temporary).

- **Jetson is slow or runs out of memory**  
  Use `ros-humble-ros-base` instead of `ros-humble-desktop`, and avoid running too many heavy nodes at once.

- **Package not found (e.g. arm64)**  
  ROS2 Humble supports Ubuntu 22.04 arm64 (Jetson). If a package is missing, ensure you ran `sudo apt update` after adding the ROS2 repo.

- **Build fails with “Package not found”**  
  Install: `sudo apt install -y ros-dev-tools python3-colcon-common-extensions`

---

## Summary checklist

- [ ] Ubuntu 22.04 on Jetson  
- [ ] Locale set to UTF-8  
- [ ] Universe repo enabled  
- [ ] ROS2 apt source installed  
- [ ] `sudo apt upgrade` done  
- [ ] `ros-humble-desktop` or `ros-humble-ros-base` installed  
- [ ] `ros-dev-tools` (and optionally colcon/rosdep) installed  
- [ ] `source /opt/ros/humble/setup.bash` works (and optionally in `.bashrc`)  
- [ ] Talker/listener demo works  
- [ ] Workspace `~/ros2_ws` created and built with colcon  
- [ ] (Optional) YDLidar SDK and ydlidar_ros2_driver built; udev rule and USB port set for G4  

After this, you can run the YDLidar G4 driver and the lidar_tools nodes as described in the main [README](../README.md).
