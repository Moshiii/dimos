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

"""Gripper adapter registry with lazy manifest discovery.

Gripper subpackages declare factories in ``_registry.py`` manifests
(see ``dimos.hardware.adapter_registry``), one folder per device —
an SDK-less device like the H100 carries its own driver and transport
alongside its adapter.

Usage:
    from dimos.hardware.grippers.registry import gripper_adapter_registry

    adapter = gripper_adapter_registry.create("mock", dof=6)
    print(gripper_adapter_registry.available())  # ["mock", ...]
"""

from __future__ import annotations

from dimos.hardware.adapter_registry import LazyAdapterRegistry
from dimos.hardware.grippers.spec import GripperAdapter


class GripperAdapterRegistry(LazyAdapterRegistry[GripperAdapter]):
    """Registry for standalone gripper adapters."""

    kind = "gripper adapter"
    manifest_roots = (("dimos.hardware.grippers", 1),)


gripper_adapter_registry = GripperAdapterRegistry()
