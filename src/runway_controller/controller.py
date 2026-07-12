"""Runway-center estimation with bounded visual and odometry feedback."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Optional

from .models import ControlCommand, ControlSource, ControllerDiagnostics, LineSegment


@dataclass(frozen=True)
class ControllerConfig:
    forward_speed_ratio: float = 0.80
    visual_heading_gain: float = 0.42
    visual_lateral_gain: float = 0.00045
    odometry_heading_gain: float = 0.30
    max_visual_rotation: float = 0.10
    max_odometry_rotation: float = 0.06
    max_lateral: float = 0.08
    runway_width_mm: float = 1000.0
    look_ahead_mm: float = 1100.0
    min_line_length_mm: float = 500.0
    min_line_forward_x_mm: float = 250.0
    min_runway_width_mm: float = 650.0
    max_runway_width_mm: float = 1350.0
    visual_timeout_ms: int = 1000
    error_filter_alpha: float = 0.30
    visual_heading_deadband_rad: float = math.radians(3.0)
    odometry_heading_deadband_rad: float = math.radians(4.0)
    max_runway_line_angle_rad: float = math.radians(32.0)


@dataclass(frozen=True)
class _Candidate:
    angle: float
    y_at_look_ahead: float
    length: float
    score: float


class RunwayController:
    """Estimate a runway centerline and generate relative walking commands."""

    def __init__(self, config: ControllerConfig = ControllerConfig()):
        self.config = config
        self.target_odometry_heading = 0.0
        self.started = False
        self.filtered_visual_heading = 0.0
        self.filtered_visual_lateral = 0.0
        self.last_runway_heading = 0.0
        self.last_runway_lateral = 0.0
        self.last_runway_confidence = 0.0
        self.last_runway_timestamp_ms: Optional[int] = None

    def start(self, odometry_heading_rad: float) -> None:
        self.target_odometry_heading = wrap_angle(odometry_heading_rad)
        self.started = True
        self.filtered_visual_heading = 0.0
        self.filtered_visual_lateral = 0.0
        self.last_runway_timestamp_ms = None

    def update(
        self,
        lines: Iterable[LineSegment],
        odometry_heading_rad: float,
        timestamp_ms: int,
    ) -> ControllerDiagnostics:
        if not self.started:
            self.start(odometry_heading_rad)

        best_left: Optional[_Candidate] = None
        best_right: Optional[_Candidate] = None
        usable_lines = 0
        for line in lines:
            candidate = self._candidate(line)
            if candidate is None:
                continue
            usable_lines += 1
            if candidate.y_at_look_ahead > 0.0:
                if best_left is None or candidate.score > best_left.score:
                    best_left = candidate
            elif best_right is None or candidate.score > best_right.score:
                best_right = candidate

        visual = self._measure_runway(best_left, best_right)
        if visual is not None:
            heading, lateral, width, confidence, both_sides = visual
            self.last_runway_timestamp_ms = int(timestamp_ms)
            self.last_runway_heading = heading
            self.last_runway_lateral = lateral
            self.last_runway_confidence = confidence
            alpha = self.config.error_filter_alpha
            self.filtered_visual_heading = wrap_angle(
                self.filtered_visual_heading * (1.0 - alpha) + heading * alpha
            )
            self.filtered_visual_lateral = (
                self.filtered_visual_lateral * (1.0 - alpha) + lateral * alpha
            )
            source = ControlSource.RUNWAY_BOTH if both_sides else ControlSource.RUNWAY_SINGLE
        elif (
            self.last_runway_timestamp_ms is not None
            and timestamp_ms - self.last_runway_timestamp_ms <= self.config.visual_timeout_ms
        ):
            heading = self.last_runway_heading
            lateral = self.last_runway_lateral
            width = self.config.runway_width_mm
            confidence = self.last_runway_confidence * 0.7
            source = ControlSource.RUNWAY_MEMORY
        else:
            heading = 0.0
            lateral = 0.0
            width = 0.0
            confidence = 0.0
            source = ControlSource.ODOMETRY

        if source is ControlSource.ODOMETRY:
            odometry_error = wrap_angle(odometry_heading_rad - self.target_odometry_heading)
            rotation = 0.0
            if abs(odometry_error) > self.config.odometry_heading_deadband_rad:
                rotation = -self.config.odometry_heading_gain * odometry_error
            rotation = clamp(rotation, self.config.max_odometry_rotation)
            lateral_command = 0.0
            reported_heading = odometry_error
            reported_lateral = 0.0
        else:
            used_heading = self.filtered_visual_heading
            used_lateral = self.filtered_visual_lateral
            rotation = 0.0
            if abs(used_heading) > self.config.visual_heading_deadband_rad:
                rotation = self.config.visual_heading_gain * used_heading
            rotation += self.config.visual_lateral_gain * used_lateral
            rotation *= min(1.0, max(0.35, confidence))
            rotation = clamp(rotation, self.config.max_visual_rotation)
            lateral_command = clamp(
                self.config.visual_lateral_gain * used_lateral * 0.5,
                self.config.max_lateral,
            )
            reported_heading = heading
            reported_lateral = lateral

        command = ControlCommand(
            forward_ratio=self.config.forward_speed_ratio,
            lateral_ratio=lateral_command,
            rotation_ratio=rotation,
        )
        return ControllerDiagnostics(
            source=source,
            usable_lines=usable_lines,
            left_detected=best_left is not None,
            right_detected=best_right is not None,
            measured_width_mm=width,
            heading_error_rad=reported_heading,
            lateral_error_mm=reported_lateral,
            confidence=confidence,
            command=command,
        )

    def _candidate(self, line: LineSegment) -> Optional[_Candidate]:
        config = self.config
        if line.length < config.min_line_length_mm:
            return None
        angle = normalize_forward_angle(line.angle)
        if abs(angle) > config.max_runway_line_angle_rad:
            return None
        if line.midpoint_x < config.min_line_forward_x_mm:
            return None

        y = line.y_at_x(config.look_ahead_mm)
        if abs(y) < 120.0:
            return None
        expected_y = config.runway_width_mm * (0.5 if y > 0.0 else -0.5)
        heading_score = 1.0 - min(1.0, abs(angle) / config.max_runway_line_angle_rad)
        side_score = 1.0 - min(1.0, abs(y - expected_y) / config.runway_width_mm)
        forward_score = min(
            1.0,
            max(0.0, (line.midpoint_x - config.min_line_forward_x_mm) / 1200.0),
        )
        score = line.length * (0.6 + 0.4 * heading_score) * (
            0.5 + 0.4 * side_score + 0.1 * forward_score
        )
        return _Candidate(angle=angle, y_at_look_ahead=y, length=line.length, score=score)

    def _measure_runway(
        self,
        left: Optional[_Candidate],
        right: Optional[_Candidate],
    ) -> Optional[tuple[float, float, float, float, bool]]:
        config = self.config
        if left is not None and right is not None:
            width = left.y_at_look_ahead - right.y_at_look_ahead
            if config.min_runway_width_mm <= width <= config.max_runway_width_mm:
                heading = wrap_angle((left.angle + right.angle) * 0.5)
                lateral = (left.y_at_look_ahead + right.y_at_look_ahead) * 0.5
                width_score = 1.0 - min(
                    1.0,
                    abs(width - config.runway_width_mm) / (config.runway_width_mm * 0.5),
                )
                return heading, lateral, width, 0.75 + 0.25 * width_score, True

        if left is not None:
            return (
                left.angle,
                left.y_at_look_ahead - config.runway_width_mm * 0.5,
                config.runway_width_mm,
                0.55,
                False,
            )
        if right is not None:
            return (
                right.angle,
                right.y_at_look_ahead + config.runway_width_mm * 0.5,
                config.runway_width_mm,
                0.55,
                False,
            )
        return None


def wrap_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def normalize_forward_angle(angle: float) -> float:
    value = wrap_angle(angle)
    if value > math.pi * 0.5:
        value -= math.pi
    elif value < -math.pi * 0.5:
        value += math.pi
    return value


def clamp(value: float, limit: float) -> float:
    return min(limit, max(-limit, value))
