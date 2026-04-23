"""ROS-agnostic contracts for navigation algorithms."""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, Dict, List


@dataclass(frozen=True)
class LidarObservation:
    """Normalized LiDAR scan data exposed to algorithms."""

    ranges_m: List[float]
    angles_rad: List[float]
    angle_min_rad: float
    angle_max_rad: float
    angle_increment_rad: float
    range_min_m: float
    range_max_m: float
    scan_time_s: float


@dataclass(frozen=True)
class VelocityCommand:
    """Algorithm output command in robot frame."""

    linear_x_m_s: float
    angular_z_rad_s: float


class NavigationAlgorithm(ABC):
    """Base class for navigation algorithms loaded by tb3_nav."""

    name: str = ""

    def configure(self, params: Dict[str, Any]) -> None:
        """Optional one-time algorithm configuration from `algo.*` params."""

    @abstractmethod
    def compute(self, obs: LidarObservation) -> VelocityCommand:
        """Return robot command from LiDAR observation."""

