"""Example plugin algorithm for developer guidance.

Behavior:
- Drives forward at configured speed.
- Monitors a forward field of view (FOV).
- If an obstacle is closer than safety distance, turns away from the closest beam.
"""

from __future__ import annotations

import math

from tb3_tools.algorithm_api import LidarObservation, NavigationAlgorithm, VelocityCommand


def _wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class ExampleReactiveAlgorithm(NavigationAlgorithm):
    """Reference algorithm showing how to integrate custom logic."""

    name = "example_reactive"

    def configure(self, params: dict) -> None:
        self.linear_x = float(params.get("linear_x", 0.05))
        self.max_angular_z = float(params.get("max_angular_z", 0.8))
        self.kp = float(params.get("kp", 1.0))
        self.safety_distance_m = float(params.get("safety_distance_m", 0.25))
        self.fov_deg = float(params.get("fov_deg", 120.0))
        self.forward_lidar_angle_deg = float(params.get("forward_lidar_angle_deg", 120.0))

    def compute(self, obs: LidarObservation) -> VelocityCommand:
        if not obs.ranges_m:
            return VelocityCommand(0.0, 0.0)

        forward_offset = math.radians(self.forward_lidar_angle_deg)
        half_fov = math.radians(self.fov_deg) / 2.0

        closest_range = float("inf")
        closest_angle = 0.0
        found = False

        for r, raw_angle in zip(obs.ranges_m, obs.angles_rad):
            if math.isnan(r) or math.isinf(r):
                continue
            if r < obs.range_min_m or r > obs.range_max_m:
                continue

            # Convert to robot-forward frame so 0 rad means robot +X.
            angle = _wrap_pi(raw_angle - forward_offset)
            if abs(angle) > half_fov:
                continue

            if r < closest_range:
                closest_range = r
                closest_angle = angle
                found = True

        if not found:
            return VelocityCommand(self.linear_x, 0.0)

        if closest_range > self.safety_distance_m:
            return VelocityCommand(self.linear_x, 0.0)

        # Turn away from the closest obstacle.
        angular = -self.kp * closest_angle
        angular = max(-self.max_angular_z, min(self.max_angular_z, angular))
        cautious_v = max(0.0, self.linear_x * 0.3)
        return VelocityCommand(cautious_v, angular)

