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

"""InFrame — a rerun archetype bound to the tf frame its entity is expressed in."""

from __future__ import annotations

from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from rerun._baseclasses import Archetype, DescribedComponentBatch


class InFrame:
    """Archetype(s) + a tf-frame attach, logged together as one entity.

    ``rr.log`` accepts this wherever an archetype goes (AsComponents protocol),
    so a ``to_rerun()`` returning it carries its own tf binding — no consumer-side
    transform logging needed. Binds via ``Transform3D(parent_frame=...)`` rather
    than ``CoordinateFrame``: as of rerun 0.32-0.35 a Pinhole sharing an entity
    with a CoordinateFrame does not resolve, while the transform attach works
    for every archetype.
    """

    def __init__(self, *archetypes: Any, frame_id: str) -> None:
        self.archetypes = archetypes
        self.frame_id = frame_id

    def as_component_batches(self) -> list[DescribedComponentBatch]:
        import rerun as rr

        attach = rr.Transform3D(parent_frame=f"tf#/{self.frame_id}")
        return [b for a in (*self.archetypes, attach) for b in a.as_component_batches()]

    def __repr__(self) -> str:
        return f"InFrame({', '.join(map(repr, self.archetypes))}, frame_id={self.frame_id!r})"


def framed(archetype: Archetype, frame_id: str, enabled: bool = True) -> Archetype | InFrame:
    """Wrap ``archetype`` in :class:`InFrame` when ``frame_id`` is set (and ``enabled``)."""
    return InFrame(archetype, frame_id=frame_id) if enabled and frame_id else archetype
