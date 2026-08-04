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

"""Fast scene asset inspection for cook reports and budget checks."""

from __future__ import annotations

from dataclasses import asdict, dataclass
import json
from pathlib import Path
from typing import Any

import numpy as np

from dimos.experimental.scene_cooking.source_assets.glb import read_glb


@dataclass(frozen=True)
class SceneAssetStats:
    path: str
    bytes: int
    format: str
    mesh_count: int = 0
    node_count: int = 0
    material_count: int = 0
    texture_count: int = 0
    vertex_count: int = 0
    triangle_count: int = 0
    primitive_count: int = 0
    draw_count: int = 0
    instance_count: int = 0
    expanded_triangle_count: int = 0
    extensions_used: tuple[str, ...] = ()

    def to_json_dict(self) -> dict[str, Any]:
        return asdict(self)


def inspect_scene_asset(path: str | Path) -> SceneAssetStats:
    """Return lightweight geometry/material counts for a supported scene file."""
    resolved = Path(path).expanduser().resolve()
    if not resolved.exists():
        raise FileNotFoundError(f"scene asset not found: {resolved}")

    suffix = resolved.suffix.lower()
    if suffix in {".glb", ".gltf"}:
        return _inspect_gltf(resolved)
    if suffix in {".usd", ".usda", ".usdc", ".usdz"}:
        return _inspect_usd(resolved)
    return _inspect_open3d(resolved)


def _inspect_gltf(path: Path) -> SceneAssetStats:
    gltf = read_glb(path)[0] if path.suffix.lower() == ".glb" else json.loads(path.read_text())
    accessors = gltf.get("accessors", [])
    meshes = gltf.get("meshes", [])
    nodes = gltf.get("nodes", [])
    if (
        not isinstance(accessors, list)
        or not isinstance(meshes, list)
        or not isinstance(nodes, list)
    ):
        raise RuntimeError(f"invalid glTF scene structure: {path}")

    mesh_triangle_counts: list[int] = []
    mesh_primitive_counts: list[int] = []
    vertex_count = 0
    triangle_count = 0
    primitive_count = 0
    for mesh in meshes:
        primitives = mesh.get("primitives", []) if isinstance(mesh, dict) else []
        mesh_triangles = 0
        for primitive in primitives:
            if not isinstance(primitive, dict):
                continue
            attributes = primitive.get("attributes", {})
            position_index = attributes.get("POSITION") if isinstance(attributes, dict) else None
            if isinstance(position_index, int):
                vertex_count += _accessor_count(accessors, position_index, path)
            element_index = primitive.get("indices", position_index)
            element_count = (
                _accessor_count(accessors, element_index, path)
                if isinstance(element_index, int)
                else 0
            )
            triangles = _triangle_count(int(primitive.get("mode", 4)), element_count)
            mesh_triangles += triangles
            triangle_count += triangles
            primitive_count += 1
        mesh_triangle_counts.append(mesh_triangles)
        mesh_primitive_counts.append(len(primitives))

    node_count = 0
    draw_count = 0
    instance_count = 0
    expanded_triangle_count = 0
    for node in nodes:
        mesh_index = node.get("mesh") if isinstance(node, dict) else None
        if not isinstance(mesh_index, int):
            continue
        if mesh_index < 0 or mesh_index >= len(meshes):
            raise RuntimeError(f"glTF node references missing mesh {mesh_index}: {path}")
        count = _node_instance_count(node, accessors, path)
        node_count += 1
        draw_count += mesh_primitive_counts[mesh_index]
        instance_count += count
        expanded_triangle_count += mesh_triangle_counts[mesh_index] * count

    return SceneAssetStats(
        path=str(path),
        bytes=path.stat().st_size,
        format=path.suffix.lower().lstrip("."),
        mesh_count=len(meshes),
        node_count=node_count,
        material_count=len(gltf.get("materials", [])),
        texture_count=len(gltf.get("textures", [])),
        vertex_count=vertex_count,
        triangle_count=triangle_count,
        primitive_count=primitive_count,
        draw_count=draw_count,
        instance_count=instance_count,
        expanded_triangle_count=expanded_triangle_count,
        extensions_used=tuple(sorted(str(value) for value in gltf.get("extensionsUsed", []))),
    )


def _accessor_count(accessors: list[Any], index: int, path: Path) -> int:
    if index < 0 or index >= len(accessors) or not isinstance(accessors[index], dict):
        raise RuntimeError(f"glTF references missing accessor {index}: {path}")
    return int(accessors[index].get("count", 0))


def _triangle_count(mode: int, element_count: int) -> int:
    if mode == 4:
        return element_count // 3
    if mode in {5, 6}:
        return max(0, element_count - 2)
    return 0


def _node_instance_count(node: dict[str, Any], accessors: list[Any], path: Path) -> int:
    extensions = node.get("extensions", {})
    instancing = extensions.get("EXT_mesh_gpu_instancing") if isinstance(extensions, dict) else None
    if not isinstance(instancing, dict):
        return 1
    attributes = instancing.get("attributes")
    if not isinstance(attributes, dict) or not attributes:
        raise RuntimeError(f"empty EXT_mesh_gpu_instancing attributes: {path}")
    counts = {
        _accessor_count(accessors, index, path)
        for index in attributes.values()
        if isinstance(index, int)
    }
    if len(counts) != 1:
        raise RuntimeError(f"inconsistent EXT_mesh_gpu_instancing accessor counts: {path}")
    return counts.pop()


def _inspect_usd(path: Path) -> SceneAssetStats:
    try:
        from pxr import Usd, UsdGeom, UsdShade  # type: ignore[import-not-found, import-untyped]
    except ImportError as exc:
        raise ImportError("inspecting USD assets requires usd-core") from exc

    stage = Usd.Stage.Open(str(path))
    if stage is None:
        raise RuntimeError(f"could not open USD stage: {path}")

    mesh_count = 0
    vertex_count = 0
    triangle_count = 0
    materials: set[str] = set()
    textures: set[str] = set()
    for prim in stage.Traverse():
        if prim.IsA(UsdGeom.Mesh):
            mesh_count += 1
            mesh = UsdGeom.Mesh(prim)
            points_raw = mesh.GetPointsAttr().Get()
            counts_raw = mesh.GetFaceVertexCountsAttr().Get()
            points = points_raw if points_raw is not None else []
            face_counts = np.asarray(counts_raw if counts_raw is not None else [], dtype=np.int32)
            vertex_count += len(points)
            triangle_count += int(np.maximum(face_counts - 2, 0).sum())
            bound = UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial()[0]
            if bound:
                materials.add(str(bound.GetPath()))
        if prim.IsA(UsdShade.Shader):
            shader = UsdShade.Shader(prim)
            if shader.GetIdAttr().Get() == "UsdUVTexture":
                file_input = shader.GetInput("file")
                if file_input and file_input.Get() is not None:
                    textures.add(str(file_input.Get()))

    return SceneAssetStats(
        path=str(path),
        bytes=path.stat().st_size,
        format=path.suffix.lower().lstrip("."),
        mesh_count=mesh_count,
        node_count=mesh_count,
        material_count=len(materials),
        texture_count=len(textures),
        vertex_count=vertex_count,
        triangle_count=triangle_count,
    )


def _inspect_open3d(path: Path) -> SceneAssetStats:
    import open3d as o3d  # type: ignore[import-untyped]

    mesh = o3d.io.read_triangle_mesh(str(path))
    if len(mesh.triangles) == 0:
        raise RuntimeError(f"empty mesh: {path}")
    return SceneAssetStats(
        path=str(path),
        bytes=path.stat().st_size,
        format=path.suffix.lower().lstrip("."),
        mesh_count=1,
        node_count=1,
        vertex_count=len(mesh.vertices),
        triangle_count=len(mesh.triangles),
    )
