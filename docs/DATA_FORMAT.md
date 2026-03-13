# LiDAR data format: how you get the points

The YDLidar G4 driver publishes **one message per scan** on the topic `/scan`. The message type is **`sensor_msgs/LaserScan`**.

---

## 1. Raw message: `sensor_msgs/LaserScan`

You do **not** get a list of (x, y) points directly. You get **one array of distances** and **fixed angle step**. Each point is then **one index** in that array.

| Field | Type | Meaning |
|-------|------|--------|
| `header.stamp` | time | When this scan was taken (timestamp) |
| `header.frame_id` | string | Coordinate frame (e.g. `"laser_frame"`) |
| `angle_min` | float (radians) | Angle of the **first** ray |
| `angle_max` | float (radians) | Angle of the **last** ray |
| `angle_increment` | float (radians) | Angle step between two consecutive rays |
| `range_min` | float (m) | Minimum valid range (e.g. 0.12 for G4) |
| `range_max` | float (m) | Maximum valid range (e.g. 16.0 for G4) |
| `ranges` | float[] | **One distance per ray** (in meters). Length = number of rays. |
| `intensities` | float[] | (Optional) One intensity per ray. May be empty. |

So the **only array of “measurements”** is **`ranges`**: each element is the **distance** (in meters) for one ray. The **angle** of that ray is computed from the index.

---

## 2. How you get each point (formula)

For each index **`i`** in `msg.ranges`:

1. **Distance (range):**
   ```text
   r = msg.ranges[i]   # in meters
   ```
   If `r` is `inf`, `nan`, or outside `[range_min, range_max]`, that ray is invalid (no hit).

2. **Angle (radians):**
   ```text
   angle = msg.angle_min + i * msg.angle_increment
   ```
   So:
   - `i = 0` → `angle = angle_min`
   - `i = 1` → `angle = angle_min + angle_increment`
   - etc.

3. **Convert to Cartesian (X, Y) in meters** (LiDAR at origin, X forward, Y left in ROS):
   ```text
   x = r * cos(angle)
   y = r * sin(angle)
   ```

So **one point** = one index `i` → you get `(angle, r)` → then `(x, y)`.

---

## 3. Summary: “form of data”

- **Input:** one `LaserScan` message per scan.
- **Per-ray data:** you have **one number per ray**: `ranges[i]` (distance in m). Angles are **not** stored; they are **computed** from `angle_min`, `angle_increment`, and index `i`.
- **One point** = index `i` → polar `(angle_i, range_i)` → Cartesian `(x_i, y_i)` in meters.

So the “form” of the points is: **polar (angle, range)** from the message, which you convert to **(x, y)** for a top-down map.

---

## 4. Example in code (Python)

```python
from sensor_msgs.msg import LaserScan
import math

def scan_callback(msg: LaserScan):
    for i, r in enumerate(msg.ranges):
        if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
            continue
        angle = msg.angle_min + i * msg.angle_increment
        x = r * math.cos(angle)
        y = r * math.sin(angle)
        # Now (x, y) is one point in meters, LiDAR at (0, 0)
        print(f"Point {i}: angle={math.degrees(angle):.2f}° range={r:.3f} m  ->  x={x:.3f} y={y:.3f}")
```

---

## 5. Typical G4 values (for orientation)

- **angle_min**: often about -π (-180°)  
- **angle_max**: about +π (180°)  
- **angle_increment**: small, e.g. ~0.001–0.01 rad, so **hundreds to thousands of points** per scan  
- **ranges**: length = number of rays; each value in **[0.12, 16.0]** meters (or `inf` if no echo)

So the **form of data** you get is: **one array of distances** (`ranges`) + **fixed angle parameters**; you derive angles from the index and then **(x, y)** from **(angle, range)**.
