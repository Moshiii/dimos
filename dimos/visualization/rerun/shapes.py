# Copyright 2026 Dimensional Inc.
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

"""Simple parametric bodies to hang off a tf frame via the bridge's ``models`` config.

A shape is a factory taking the ``rerun`` module (so blueprints can name a shape
without importing rerun) and returning ``(subpath, archetype)`` parts. Geometry is
in the frame's own axes — FLU, X forward — and gets logged statically; the frame
moves it.

Factories ride in a module config to the bridge's own worker process, so they must
pickle: bind parameters with ``partial`` over a module-level builder, never a closure.
"""

from __future__ import annotations

from collections.abc import Callable
from functools import partial
from typing import Any, TypeAlias

import numpy as np

ShapePart: TypeAlias = tuple[str, Any]
ShapeFactory: TypeAlias = Callable[[Any], list[ShapePart]]

WHITE = (255, 255, 255)


def quadcopter(
    arm_m: float = 2.0,
    color: tuple[int, int, int] = (255, 90, 60),
    radius_m: float = 0.12,
) -> ShapeFactory:
    """X-config quadcopter: arms, rotor discs, a white nose marker and a hub box.

    ``arm_m`` is hub-to-rotor, so the tip-to-tip span is twice that. The default is
    city scale — big enough to stay readable against a kilometre of mesh.
    """
    return partial(_quadcopter, arm_m=arm_m, color=color, radius_m=radius_m)


def _quadcopter(
    rr: Any,
    arm_m: float,
    color: tuple[int, int, int],
    radius_m: float,
) -> list[ShapePart]:
    d = arm_m / np.sqrt(2)
    arms = [
        np.array([[-d, -d, 0], [d, d, 0]], dtype=np.float32),
        np.array([[-d, d, 0], [d, -d, 0]], dtype=np.float32),
    ]
    theta = np.linspace(0, 2 * np.pi, 24, dtype=np.float32)
    r = arm_m * 0.35
    circle = np.stack([r * np.cos(theta), r * np.sin(theta), np.zeros_like(theta)], axis=-1)
    rotors = [
        circle + np.array([sx * d, sy * d, 0], dtype=np.float32) for sx in (-1, 1) for sy in (-1, 1)
    ]
    # Nose marker so yaw is readable at a glance.
    nose = [np.array([[0, 0, 0], [arm_m * 0.8, 0, 0]], dtype=np.float32)]

    return [
        (
            "frame",
            rr.LineStrips3D(
                arms + rotors + nose,
                colors=[color] * 6 + [WHITE],
                radii=[radius_m],
            ),
        ),
        (
            "hub",
            rr.Boxes3D(
                centers=[[0, 0, 0]],
                half_sizes=[[arm_m * 0.18, arm_m * 0.18, arm_m * 0.08]],
                colors=[color],
            ),
        ),
    ]
