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

from __future__ import annotations

import numpy as np

from dimos.msgs.visualization_msgs.EntityMesh import EntityMesh


def _mesh() -> EntityMesh:
    return EntityMesh(
        path="world/city/tiles/3_-4/buildings",
        frame_id="enu",
        vertices=np.arange(9, dtype=np.float32).reshape(3, 3),
        triangles=np.array([[0, 1, 2]], dtype=np.uint32),
        colors=np.tile(np.array([[20, 40, 80, 120]], dtype=np.uint8), (3, 1)),
        ts=1234.5,
    )


def test_roundtrip_preserves_everything():
    m = _mesh()
    d = EntityMesh.decode(m.encode())
    assert d.path == m.path
    assert d.frame_id == m.frame_id
    assert d.op == "set"
    assert d.ts == m.ts
    np.testing.assert_array_equal(d.vertices, m.vertices)
    np.testing.assert_array_equal(d.triangles, m.triangles)
    np.testing.assert_array_equal(d.colors, m.colors)


def test_rgb_colors_roundtrip_as_opaque_rgba():
    """Terrain meshes carry (N, 3) RGB; the wire format is fixed at RGBA."""
    m = EntityMesh(
        path="x",
        vertices=np.zeros((3, 3)),
        triangles=np.array([[0, 1, 2]]),
        colors=np.tile(np.array([[10, 20, 30]], dtype=np.uint8), (3, 1)),
    )
    d = EntityMesh.decode(m.encode())
    assert d.colors is not None
    assert d.colors.shape == (3, 4)
    assert list(d.colors[0]) == [10, 20, 30, 255]


def test_roundtrip_without_colors():
    m = EntityMesh(path="x", vertices=np.zeros((2, 3)), triangles=np.zeros((0, 3)))
    d = EntityMesh.decode(m.encode())
    assert d.colors is None
    assert len(d.vertices) == 2


def test_clear_roundtrip():
    d = EntityMesh.decode(EntityMesh.clear("world/city/tiles/0_0", ts=7.0).encode())
    assert d.op == "clear"
    assert d.path == "world/city/tiles/0_0"
    assert d.ts == 7.0
    assert len(d.vertices) == 0


def test_edges_roundtrip_and_render_as_sibling_wireframe():
    m = _mesh()
    m.edges = [np.arange(6, dtype=np.float32).reshape(2, 3)]
    m.edge_color = (10, 20, 30)
    m.edge_radius = 0.5
    d = EntityMesh.decode(m.encode())
    assert d.edge_color == (10, 20, 30)
    assert d.edge_radius == 0.5
    np.testing.assert_array_equal(d.edges[0], m.edges[0])

    paths = [p for p, _ in d.to_rerun()]
    assert f"{m.path}/edges" in paths


def test_decode_tolerates_edgeless_old_wire_format():
    """Recordings predate the edges block; decode must not require it."""
    m = _mesh()
    old_bytes = m.encode()[:-4]  # strip the (empty) edges block
    d = EntityMesh.decode(old_bytes)
    assert d.edges == []
    np.testing.assert_array_equal(d.vertices, m.vertices)


def test_to_rerun_set_carries_mesh_and_frame_attachment():
    out = _mesh().to_rerun()
    paths = [p for p, _ in out]
    kinds = {type(a).__name__ for _, a in out}
    assert paths == ["world/city/tiles/3_-4/buildings"] * 2
    assert kinds == {"Mesh3D", "Transform3D"}, "the reparent onto tf#/enu must ride along"


def test_to_rerun_moves_alpha_onto_the_material():
    """The viewer's translucent pipeline keys off albedo_factor alpha; vertex
    alpha alone renders solid."""
    (_, mesh3d), _ = _mesh().to_rerun()
    assert mesh3d.albedo_factor is not None


def test_to_rerun_without_frame_is_just_the_mesh():
    m = EntityMesh(path="x", vertices=np.zeros((3, 3)), triangles=np.zeros((1, 3)))
    out = m.to_rerun()
    assert len(out) == 1
    assert type(out[0][1]).__name__ == "Mesh3D"


def test_to_rerun_clear():
    out = EntityMesh.clear("world/city/tiles/0_0").to_rerun()
    assert len(out) == 1
    assert out[0][0] == "world/city/tiles/0_0"
    assert type(out[0][1]).__name__ == "Clear"
