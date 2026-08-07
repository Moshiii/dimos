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

"""Label colors for rerun overlays, drawn from the shared vis palette."""

from __future__ import annotations

import hashlib

from dimos.memory2.vis.color import PALETTE


def color_for_label(label: str) -> tuple[int, int, int]:
    """One color per label, stable across runs and processes."""
    digest = hashlib.blake2b(label.encode(), digest_size=8).digest()
    return PALETTE[int.from_bytes(digest, "big") % len(PALETTE)].rgb_u8()
