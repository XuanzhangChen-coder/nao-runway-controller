"""Data types shared by the controller, synthetic input, and tests."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum


class ControlSource(str, Enum):
    RUNWAY_BOTH = "runway_both"
    RUNWAY_SINGLE = "runway_single"
    RUNWAY_MEMORY = "runway_memory"
    ODOMETRY = "odometry"


@dataclass(frozen=True)
class LineSegment:
    """A detected ground-plane line in robot coordinates, measured in mm."""

    first_x: float
    first_y: float
    last_x: float
    last_y: float

    @property
    def length(self) -> float:
        return math.hypot(self.last_x - self.first_x, self.last_y - self.first_y)

    @property
    def midpoint_x(self) -> float:
        return (self.first_x + self.last_x) * 0.5

    @property
    def angle(self) -> float:
        return math.atan2(self.last_y - self.first_y, self.last_x - self.first_x)

    def y_at_x(self, x: float) -> float:
        dx = self.last_x - self.first_x
        if abs(dx) < 1.0:
            return (self.first_y + self.last_y) * 0.5
        return self.first_y + (self.last_y - self.first_y) * (x - self.first_x) / dx


@dataclass(frozen=True)
class ControlCommand:
    """Unitless relative walking request, not a velocity in m/s."""

    forward_ratio: float
    lateral_ratio: float
    rotation_ratio: float


@dataclass(frozen=True)
class ControllerDiagnostics:
    source: ControlSource
    usable_lines: int
    left_detected: bool
    right_detected: bool
    measured_width_mm: float
    heading_error_rad: float
    lateral_error_mm: float
    confidence: float
    command: ControlCommand
