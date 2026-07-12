"""Chest-button stand/walk state machine from the original demonstration."""

from __future__ import annotations

from enum import Enum


class DemoMode(str, Enum):
    STAND = "stand"
    RUNNING = "running"


class RunDemoStateMachine:
    """Toggle mode once per button press edge."""

    def __init__(self) -> None:
        self.mode = DemoMode.STAND
        self._pressed_last_update = False

    def update(self, chest_button_pressed: bool) -> DemoMode:
        pressed = bool(chest_button_pressed)
        if pressed and not self._pressed_last_update:
            self.mode = (
                DemoMode.RUNNING if self.mode is DemoMode.STAND else DemoMode.STAND
            )
        self._pressed_last_update = pressed
        return self.mode

    def reset(self) -> None:
        self.mode = DemoMode.STAND
        self._pressed_last_update = False
