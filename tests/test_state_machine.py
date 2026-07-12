from __future__ import annotations

import sys
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from runway_controller import DemoMode, RunDemoStateMachine


class StateMachineTest(unittest.TestCase):
    def test_one_toggle_per_press_edge(self):
        state = RunDemoStateMachine()
        self.assertEqual(state.mode, DemoMode.STAND)
        self.assertEqual(state.update(True), DemoMode.RUNNING)
        self.assertEqual(state.update(True), DemoMode.RUNNING)
        self.assertEqual(state.update(False), DemoMode.RUNNING)
        self.assertEqual(state.update(True), DemoMode.STAND)


if __name__ == "__main__":
    unittest.main()
