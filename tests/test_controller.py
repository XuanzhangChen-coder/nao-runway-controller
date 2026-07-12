from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from runway_controller import ControlSource, LineSegment, RunwayController
from runway_controller.synthetic import runway_lines, simulate_convergence


class RunwayControllerTest(unittest.TestCase):
    def test_two_boundaries_reconstruct_centerline(self):
        controller = RunwayController()
        result = controller.update(
            runway_lines(math.radians(8.0), 180.0),
            odometry_heading_rad=0.0,
            timestamp_ms=0,
        )
        self.assertEqual(result.source, ControlSource.RUNWAY_BOTH)
        self.assertAlmostEqual(result.measured_width_mm, 1000.0, places=6)
        self.assertAlmostEqual(result.lateral_error_mm, 180.0, places=6)
        self.assertTrue(result.left_detected)
        self.assertTrue(result.right_detected)
        self.assertLessEqual(abs(result.command.rotation_ratio), 0.10)
        self.assertLessEqual(abs(result.command.lateral_ratio), 0.08)

    def test_single_boundary_estimates_center(self):
        controller = RunwayController()
        result = controller.update(
            runway_lines(0.0, -120.0, include_right=False),
            odometry_heading_rad=0.0,
            timestamp_ms=0,
        )
        self.assertEqual(result.source, ControlSource.RUNWAY_SINGLE)
        self.assertAlmostEqual(result.lateral_error_mm, -120.0)

    def test_memory_then_odometry_fallback(self):
        controller = RunwayController()
        controller.update(runway_lines(0.1, 100.0), 0.0, 0)
        memory = controller.update([], 0.0, 900)
        odometry = controller.update([], 0.2, 1101)
        self.assertEqual(memory.source, ControlSource.RUNWAY_MEMORY)
        self.assertEqual(odometry.source, ControlSource.ODOMETRY)
        self.assertLess(odometry.command.rotation_ratio, 0.0)

    def test_short_or_transverse_lines_are_rejected(self):
        controller = RunwayController()
        lines = [
            LineSegment(300.0, 500.0, 400.0, 500.0),
            LineSegment(500.0, -600.0, 500.0, 600.0),
        ]
        result = controller.update(lines, 0.0, 0)
        self.assertEqual(result.source, ControlSource.ODOMETRY)
        self.assertEqual(result.usable_lines, 0)

    def test_synthetic_errors_converge(self):
        history = simulate_convergence(RunwayController(), steps=180)
        self.assertLess(abs(history[-1].heading_error_rad), math.radians(3.2))
        self.assertLess(abs(history[-1].lateral_error_mm), 35.0)


if __name__ == "__main__":
    unittest.main()
