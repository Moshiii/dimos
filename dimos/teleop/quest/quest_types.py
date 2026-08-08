#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Quest controller types with nice API for parsing Joy messages."""

from dataclasses import dataclass, field
from enum import IntEnum
from typing import ClassVar

from dimos.msgs.sensor_msgs.Joy import Joy
from dimos.teleop.types import TeleopControls


class Hand(IntEnum):
    LEFT = 0
    RIGHT = 1


@dataclass
class ThumbstickState:
    """State of a thumbstick with X/Y axes."""

    x: float = 0.0
    y: float = 0.0


@dataclass
class QuestControllerState:
    """Parsed Quest controller state from Joy message with no data loss.

    Preserves full-fidelity analog values (trigger, grip as floats, thumbstick axes)
    from the raw Joy message in a readable format. Use this when you need analog
    precision (e.g., proportional grip control). Subclasses can publish this
    alongside TeleopControls for float access.

    Axes layout:
        0: thumbstick X, 1: thumbstick Y, 2: trigger (analog), 3: grip (analog)
    Button indices (digital, 0 or 1):
        0: trigger, 1: grip, 2: touchpad, 3: thumbstick,
        4: X/A, 5: Y/B, 6: menu
    """

    EXPECTED_AXES: ClassVar[int] = 4
    EXPECTED_BUTTONS: ClassVar[int] = 7

    is_left: bool = True
    # Analog values (0.0-1.0)
    trigger: float = 0.0
    grip: float = 0.0
    # Digital buttons
    touchpad: bool = False
    thumbstick_press: bool = False
    primary: bool = False  # X on left, A on right
    secondary: bool = False  # Y on left, B on right
    menu: bool = False
    # Thumbstick axes
    thumbstick: ThumbstickState = field(default_factory=ThumbstickState)

    @classmethod
    def from_joy(cls, joy: Joy, is_left: bool = True) -> "QuestControllerState":
        """Create QuestControllerState from Joy message.
        Expected axes: [thumbstick_x, thumbstick_y, trigger_analog, grip_analog]
        Expected buttons: [trigger, grip, touchpad, thumbstick, X/A, Y/B, menu]
        Raises:
            ValueError: If Joy message doesn't have expected Quest controller format.
        """
        buttons = joy.buttons or []
        axes = joy.axes or []

        if len(buttons) < cls.EXPECTED_BUTTONS:
            raise ValueError(f"Expected {cls.EXPECTED_BUTTONS} buttons, got {len(buttons)}")
        if len(axes) < cls.EXPECTED_AXES:
            raise ValueError(f"Expected {cls.EXPECTED_AXES} axes, got {len(axes)}")

        return cls(
            is_left=is_left,
            trigger=float(axes[2]),
            grip=float(axes[3]),
            touchpad=buttons[2] > 0.5,
            thumbstick_press=buttons[3] > 0.5,
            primary=buttons[4] > 0.5,
            secondary=buttons[5] > 0.5,
            menu=buttons[6] > 0.5,
            thumbstick=ThumbstickState(x=float(axes[0]), y=float(axes[1])),
        )


def teleop_controls_from_controllers(
    left: QuestControllerState | None,
    right: QuestControllerState | None,
) -> TeleopControls:
    """Normalize Quest controller state into the common teleop message."""
    controls = TeleopControls()
    for side, controller in (("left", left), ("right", right)):
        if controller is None:
            continue
        controls.set_attribute(f"{side}_trigger", controller.trigger > 0.5)
        controls.set_attribute(f"{side}_grip", controller.grip > 0.5)
        controls.set_attribute(f"{side}_touchpad", controller.touchpad)
        controls.set_attribute(f"{side}_thumbstick", controller.thumbstick_press)
        controls.set_attribute(f"{side}_primary", controller.primary)
        controls.set_attribute(f"{side}_secondary", controller.secondary)
        controls.set_attribute(f"{side}_menu", controller.menu)
    controls.pack_analog_triggers(left.trigger if left else 0.0, right.trigger if right else 0.0)
    return controls


# Quest controller face-button labels to common control names. Callers can
# also pass a raw attribute name (e.g. "right_grip") directly where an alias is
# accepted.
BUTTON_ALIASES: dict[str, str] = {
    "A": "right_primary",
    "B": "right_secondary",
    "X": "left_primary",
    "Y": "left_secondary",
    "LT": "left_trigger",
    "RT": "right_trigger",
    "LG": "left_grip",
    "RG": "right_grip",
    "MENU_L": "left_menu",
    "MENU_R": "right_menu",
}
