"""Deterministic synthetic runway observations for local verification."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable

from .controller import RunwayController, wrap_angle
from .models import ControllerDiagnostics, LineSegment


@dataclass(frozen=True)
class SyntheticState:
    timestamp_ms: int
    heading_error_rad: float
    lateral_error_mm: float
    diagnostics: ControllerDiagnostics


def runway_lines(
    heading_error_rad: float,
    lateral_error_mm: float,
    width_mm: float = 1000.0,
    look_ahead_mm: float = 1100.0,
    include_left: bool = True,
    include_right: bool = True,
) -> list[LineSegment]:
    """Generate two ground-plane boundary segments seen from the robot frame."""

    x_near, x_far = 300.0, 2300.0
    slope = math.tan(heading_error_rad)

    def boundary(side_offset: float) -> LineSegment:
        def y_at(x: float) -> float:
            return lateral_error_mm + side_offset + slope * (x - look_ahead_mm)

        return LineSegment(x_near, y_at(x_near), x_far, y_at(x_far))

    lines = []
    if include_left:
        lines.append(boundary(width_mm * 0.5))
    if include_right:
        lines.append(boundary(-width_mm * 0.5))
    return lines


def simulate_convergence(
    controller: RunwayController,
    steps: int = 160,
    dt_s: float = 0.05,
    initial_heading_rad: float = math.radians(11.0),
    initial_lateral_mm: float = 260.0,
    dropout_steps: Iterable[int] = (),
) -> list[SyntheticState]:
    """Run a simple error-space simulation; results are not real-robot metrics."""

    heading = float(initial_heading_rad)
    lateral = float(initial_lateral_mm)
    odometry_heading = 0.0
    dropouts = set(dropout_steps)
    history = []
    controller.start(odometry_heading)

    for step in range(steps):
        timestamp_ms = int(round(step * dt_s * 1000.0))
        lines = [] if step in dropouts else runway_lines(heading, lateral)
        diagnostics = controller.update(lines, odometry_heading, timestamp_ms)
        command = diagnostics.command

        heading = wrap_angle(heading - command.rotation_ratio * 1.6 * dt_s)
        lateral -= command.lateral_ratio * 1600.0 * dt_s
        lateral -= math.sin(heading) * 120.0 * dt_s
        odometry_heading = wrap_angle(odometry_heading + command.rotation_ratio * 0.4 * dt_s)
        history.append(SyntheticState(timestamp_ms, heading, lateral, diagnostics))
    return history
