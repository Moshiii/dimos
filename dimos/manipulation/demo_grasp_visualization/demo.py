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
from threading import Event
from typing import Protocol

import numpy as np
from numpy.typing import NDArray

from dimos.manipulation.demo_graspgenx.demo import _validate
from dimos.manipulation.demo_graspgenx.fixture import load_demo_clouds
from dimos.manipulation.demo_graspgenx.render import gripper_wireframe_geometry
from dimos.manipulation.grasping.grasp_gen_spec import GraspGenSpec
from dimos.manipulation.grasping.grasp_gen_x import (
    GraspGenXConfig,
    GraspGenXModule,
    RigidTransform,
    SweepVolumeGripperConfig,
)
from dimos.manipulation.visualization.layers import (
    LineSetElement,
    PointCloudElement,
    VisualizationLayer,
)
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.manipulation.visualization.viser.visualizer import ViserManipulationVisualizer
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.robot.manipulators.xarm.grasp_config import make_xarm_graspgenx_config

DEFAULT_MAX_CANDIDATES = 20
OBJECT_CLOUD_LAYER_ID = "grasp/object-cloud"
PROPOSAL_LAYER_ID = "grasp/proposals"

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


def _rank_color(rank_index: int, count: int) -> NDArray[np.uint8]:
    fraction = 0.0 if count <= 1 else rank_index / (count - 1)
    start = np.asarray([0, 220, 80], dtype=float)
    end = np.asarray([255, 140, 0], dtype=float)
    return np.asarray(
        np.rint(start + fraction * (end - start)),
        dtype=np.uint8,
    )


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
    proposals = proposer.propose_grasps(object_cloud)
    _validate(proposals, object_cloud)
    ranked = sorted(proposals.candidates, key=lambda candidate: candidate.score, reverse=True)
    displayed = ranked[:max_candidates]

    points, colors = object_cloud.as_numpy()
    cloud_layer = VisualizationLayer(
        OBJECT_CLOUD_LAYER_ID,
        object_cloud.frame_id,
        (PointCloudElement("object", points, colors),),
    )
    wireframes = []
    for index, candidate in enumerate(displayed):
        vertices, edges = gripper_wireframe_geometry(
            candidate,
            gripper,
            grasp_frame_to_tcp,
        )
        wireframes.append(
            LineSetElement(
                f"rank-{index + 1}",
                vertices,
                edges,
                colors=_rank_color(index, len(displayed)),
                line_width=2.5,
            )
        )
    proposal_layer = VisualizationLayer(
        PROPOSAL_LAYER_ID,
        proposals.header.frame_id,
        tuple(wireframes),
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
        result = run_demo(
            proposer,
            visualizer,
            gripper=active_config.gripper,
            grasp_frame_to_tcp=active_config.grasp_frame_to_tcp,
            max_candidates=max_candidates,
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
