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

"""Device-independent messages shared by teleoperation adapters and tasks."""

from dimos.msgs.std_msgs.UInt32 import UInt32


class TeleopControls(UInt32):
    """Packed control state for a left/right teleoperation device pair.

    Digital controls occupy bits 0-14. Analog trigger values use seven bits
    each in bits 16-29, leaving the sign bit clear for LCM's signed int32.
    """

    BITS = {
        "left_trigger": 0,
        "left_grip": 1,
        "left_touchpad": 2,
        "left_thumbstick": 3,
        "left_primary": 4,
        "left_secondary": 5,
        "left_menu": 6,
        "right_trigger": 8,
        "right_grip": 9,
        "right_touchpad": 10,
        "right_thumbstick": 11,
        "right_primary": 12,
        "right_secondary": 13,
        "right_menu": 14,
    }

    _LEFT_TRIGGER_SHIFT = 16
    _RIGHT_TRIGGER_SHIFT = 23
    _ANALOG_MASK = 0x7F
    _ANALOG_MAX = 127

    @property
    def left_trigger_analog(self) -> float:
        """Return the normalized left trigger value."""
        return ((self.data >> self._LEFT_TRIGGER_SHIFT) & self._ANALOG_MASK) / self._ANALOG_MAX

    @property
    def right_trigger_analog(self) -> float:
        """Return the normalized right trigger value."""
        return ((self.data >> self._RIGHT_TRIGGER_SHIFT) & self._ANALOG_MASK) / self._ANALOG_MAX

    def pack_analog_triggers(self, left: float, right: float) -> None:
        """Pack normalized trigger values into the transport message."""
        left_u7 = round(max(0.0, min(1.0, left)) * self._ANALOG_MAX)
        right_u7 = round(max(0.0, min(1.0, right)) * self._ANALOG_MAX)
        self.data = (
            (self.data & 0x0000FFFF)
            | (left_u7 << self._LEFT_TRIGGER_SHIFT)
            | (right_u7 << self._RIGHT_TRIGGER_SHIFT)
        )

    def __getattr__(self, name: str) -> bool:
        if name in self.BITS:
            return bool(self.data & (1 << self.BITS[name]))
        raise AttributeError(f"'{type(self).__name__}' has no attribute '{name}'")

    def _set_bit(self, name: str, value: bool) -> None:
        if value:
            self.data |= 1 << self.BITS[name]
        else:
            self.data &= ~(1 << self.BITS[name])

    def __setattr__(self, name: str, value: bool) -> None:
        if name in self.BITS:
            self._set_bit(name, value)
        else:
            super().__setattr__(name, value)

    def set_attribute(self, name: str, value: bool) -> None:
        """Set a named digital control, rejecting unknown names."""
        if name not in self.BITS:
            raise KeyError(f"unknown control {name!r}; valid controls: {sorted(self.BITS)}")
        self._set_bit(name, value)
