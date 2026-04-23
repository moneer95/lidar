# Integrate your own navigation algorithm

Follow these five steps to plug a custom planner into `tb3_nav` without touching ROS wiring (`/scan`, `/cmd_vel`, motor power).

---

## Step 1 — Add a plugin file

Create a new Python file under:

`src/algorithms/<your_name>.py`

Example: `src/algorithms/my_nav.py`

---

## Step 2 — Implement the plugin contract

1. Import the API:

   ```python
   from tb3_tools.algorithm_api import (
       NavigationAlgorithm,
       LidarObservation,
       VelocityCommand,
   )
   ```

2. Define a class that **subclasses** `NavigationAlgorithm`.

3. Set a unique string id:

   ```python
   name = "my_nav"
   ```

4. Implement:

   - **`configure(self, params: dict)`**  
     Receives values from YAML keys `algo.*` (e.g. `algo.linear_x` → `params["linear_x"]`).

   - **`compute(self, obs: LidarObservation) -> VelocityCommand`**  
     Use `obs.ranges_m` and `obs.angles_rad` (same index).  
     Return `VelocityCommand(linear_x_m_s=..., angular_z_rad_s=...)`.

Use **`src/algorithms/example_reactive.py`** as a working template.

---

## Step 3 — Select which algorithm runs

- **Option A:** Keep only your plugin in `src/algorithms/` (remove or rename `example_reactive.py` if you do not want it).

- **Option B:** If multiple plugins exist, `tb3_nav` picks the first by **alphabetical `name`**, unless you set explicitly in **`config/tb3_nav_template.yaml`**:

  ```yaml
  tb3_nav:
    ros__parameters:
      algorithm: "my_nav"
  ```

  The value must match your class’s `name` exactly.

---

## Step 4 — Tune parameters (optional)

Edit **`config/tb3_nav_template.yaml`**: under `tb3_nav` → `ros__parameters`, add or change **`algo.*`** keys that your `configure()` reads.

Example:

```yaml
algo.linear_x: 0.03
algo.max_angular_z: 0.5
```

Runner settings (topics, `invert_drive`, etc.) stay in the same file under `tb3_nav.ros__parameters`.

---

## Step 5 — Build and run

```bash
cd ~/Desktop/lidar
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
./start_robot.sh
```

Or run only navigation:

```bash
ros2 run tb3_tools tb3_nav --ros-args --params-file ~/Desktop/lidar/config/tb3_nav_template.yaml
```

---

## See also

- [README.md](../README.md) — full onboarding, TurtleBot3 + YDLidar, troubleshooting
- `src/tb3_tools/tb3_tools/algorithm_api.py` — `LidarObservation` / `VelocityCommand` fields
