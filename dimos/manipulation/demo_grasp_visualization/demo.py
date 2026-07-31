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

"""Build display-only layers from the banana fixture and real grasp proposals."""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass
from pathlib import Path
from threading import Event
from typing import Protocol

from dimos.manipulation.demo_graspgenx.demo import _validate
from dimos.manipulation.demo_graspgenx.fixture import load_demo_clouds
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.grasp_gen_x import (
    GraspGenXConfig,
    GraspGenXModule,
    RigidTransform,
    SweepVolumeGripperConfig,
)
from dimos.manipulation.grasping.grasp_proposal import GraspProposalInput
from dimos.manipulation.visualization.grasp import (
    GraspCandidateVisualState,
    VisualizedGraspCandidate,
    build_grasp_object_cloud_layer,
    build_grasp_proposals_layer,
)
from dimos.manipulation.visualization.layers import VisualizationLayer
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.visualizer import ViserManipulationVisualizer
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config

DEFAULT_MAX_CANDIDATES = 20
CloudLoader = Callable[[], tuple[PointCloud2, PointCloud2]]
Waiter = Callable[[], None]


class LayerVisualizer(Protocol):
    def set_layer(self, layer: VisualizationLayer) -> None: ...

    def get_visualization_url(self) -> str | None: ...

    def close(self) -> None: ...


@dataclass(frozen=True)
class GraspVisualizationDemoResult:
    """Summary of layers submitted by one interactive visualization run."""

    candidate_count: int
    displayed_count: int
    frame_id: str
    visualization_url: str | None


def load_object_cloud_file(
    path: Path,
    *,
    frame_id: str = "world",
) -> tuple[PointCloud2, PointCloud2]:
    """Load a segmented object cloud from a PLY or PCD file."""
    import open3d as o3d  # type: ignore[import-untyped]

    pointcloud = o3d.io.read_point_cloud(str(path))
    if pointcloud.is_empty():
        raise ValueError(f"object point cloud is empty or unreadable: {path}")
    cloud = PointCloud2(pointcloud, frame_id=frame_id, ts=0.0)
    return cloud, cloud


def run_demo(
    proposer: GraspGenSpec,
    visualizer: LayerVisualizer,
    *,
    gripper: SweepVolumeGripperConfig,
    grasp_frame_to_tcp: RigidTransform,
    max_candidates: int = DEFAULT_MAX_CANDIDATES,
    cloud_loader: CloudLoader = load_demo_clouds,
) -> GraspVisualizationDemoResult:
    """Publish banana object-cloud and ranked gripper-wireframe layers."""
    if max_candidates <= 0:
        raise ValueError("max_candidates must be positive")
    _, object_cloud = cloud_loader()
    proposals = proposer.propose_grasps(GraspProposalInput.from_pointcloud(object_cloud))
    _validate(proposals, object_cloud)
    ranked = sorted(proposals.candidates, key=lambda candidate: candidate.score, reverse=True)
    displayed = ranked[:max_candidates]

    cloud_layer = build_grasp_object_cloud_layer(object_cloud)
    proposal_layer = build_grasp_proposals_layer(
        [
            VisualizedGraspCandidate(
                candidate,
                index,
                GraspCandidateVisualState.RANKED,
            )
            for index, candidate in enumerate(displayed, start=1)
        ],
        frame_id=proposals.header.frame_id,
        gripper=gripper,
        grasp_frame_to_tcp=grasp_frame_to_tcp,
    )
    visualizer.set_layer(cloud_layer)
    visualizer.set_layer(proposal_layer)
    return GraspVisualizationDemoResult(
        candidate_count=len(ranked),
        displayed_count=len(displayed),
        frame_id=proposals.header.frame_id,
        visualization_url=visualizer.get_visualization_url(),
    )


def wait_until_interrupted() -> None:
    """Keep the interactive Viser process alive until Ctrl-C."""
    Event().wait()


def run_contributor_demo(
    *,
    max_candidates: int = DEFAULT_MAX_CANDIDATES,
    config: GraspGenXConfig | None = None,
    object_cloud_path: Path | None = None,
    object_frame_id: str = "world",
    waiter: Waiter = wait_until_interrupted,
) -> GraspVisualizationDemoResult:
    """Run real GraspGenX, publish layers, and own all interactive resources."""
    active_config = config if config is not None else make_xarm_graspgenx_config()
    proposer = GraspGenXModule(
        **active_config.model_dump(exclude={"rpc_transport", "tf_transport", "g"})
    )
    visualizer = ViserManipulationVisualizer(config=ViserVisualizationConfig(panel_enabled=False))
    proposer_started = False
    try:
        proposer.start()
        proposer_started = True
        cloud_loader = (
            load_demo_clouds
            if object_cloud_path is None
            else lambda: load_object_cloud_file(object_cloud_path, frame_id=object_frame_id)
        )
        result = run_demo(
            proposer,
            visualizer,
            gripper=active_config.gripper,
            grasp_frame_to_tcp=active_config.grasp_frame_to_tcp,
            max_candidates=max_candidates,
            cloud_loader=cloud_loader,
        )
        print(
            "grasp-visualization-demo "
            f"candidates={result.candidate_count} displayed={result.displayed_count} "
            f"url={result.visualization_url}",
            flush=True,
        )
        waiter()
        return result
    finally:
        visualizer.close()
        if proposer_started:
            proposer.stop()
