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

"""Hermetic tests for the interactive grasp visualization demo."""

from __future__ import annotations

import numpy as np
import open3d as o3d
import pytest
from pytest_mock import MockerFixture

from dimos.manipulation.demo_grasp_visualization import __main__
import dimos.manipulation.demo_grasp_visualization.demo as demo_module
from dimos.manipulation.demo_grasp_visualization.demo import (
    GraspVisualizationDemoResult,
    run_contributor_demo,
    run_demo,
)
from dimos.manipulation.demo_graspgenx.demo import deployment_config
from dimos.manipulation.grasping.grasp_proposal import GraspProposalInput
from dimos.manipulation.visualization.layers import (
    LineSetElement,
    PointCloudElement,
    VisualizationLayer,
)
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.manipulation_msgs.GraspCandidate import GraspCandidate
from dimos.msgs.manipulation_msgs.GraspCandidateArray import GraspCandidateArray
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.std_msgs.Header import Header


class Proposer:
    def __init__(self, candidates: list[GraspCandidate]) -> None:
        self.candidates = candidates
        self.calls = 0

    def propose_grasps(self, proposal_input: GraspProposalInput) -> GraspCandidateArray:
        self.calls += 1
        cloud = proposal_input.object_pointcloud
        return GraspCandidateArray(Header(float(cloud.ts), cloud.frame_id), self.candidates)


class Visualizer:
    def __init__(self) -> None:
        self.layers: list[VisualizationLayer] = []
        self.closed = False

    def set_layer(self, layer: VisualizationLayer) -> None:
        self.layers.append(layer)

    def get_visualization_url(self) -> str:
        return "http://localhost:8095"

    def close(self) -> None:
        self.closed = True


def candidate(x: float, score: float) -> GraspCandidate:
    return GraspCandidate(
        Pose(
            {
                "position": [x, 0.0, 0.2],
                "orientation": [0.0, 0.0, 0.0, 1.0],
            }
        ),
        score,
    )


def clouds() -> tuple[PointCloud2, PointCloud2]:
    points = np.asarray([[0.0, 0.0, 0.0], [0.1, 0.2, 0.3]], dtype=np.float32)
    colors = np.asarray([[1.0, 0.5, 0.0], [0.0, 1.0, 0.0]], dtype=np.float32)
    cloud = PointCloud2.from_numpy(points, frame_id="world", timestamp=42.0)
    cloud.pointcloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud, cloud


def test_demo_publishes_colored_cloud_and_limited_ranked_wireframes() -> None:
    proposer = Proposer([candidate(1.0, 0.9), candidate(2.0, 0.7), candidate(3.0, 0.5)])
    visualizer = Visualizer()
    config = deployment_config()

    result = run_demo(
        proposer,
        visualizer,
        gripper=config.gripper,
        grasp_frame_to_tcp=config.grasp_frame_to_tcp,
        max_candidates=2,
        cloud_loader=clouds,
    )

    assert result == GraspVisualizationDemoResult(3, 2, "world", "http://localhost:8095")
    assert proposer.calls == 1
    assert [layer.id for layer in visualizer.layers] == [
        "grasp/object-cloud",
        "grasp/proposals",
    ]
    cloud_element = visualizer.layers[0].elements[0]
    assert isinstance(cloud_element, PointCloudElement)
    np.testing.assert_array_equal(
        cloud_element.colors,
        np.asarray([[255, 128, 0], [0, 255, 0]], dtype=np.uint8),
    )
    proposals = visualizer.layers[1].elements
    assert [element.id for element in proposals] == ["rank-1", "rank-2"]
    assert all(isinstance(element, LineSetElement) for element in proposals)
    np.testing.assert_array_equal(proposals[0].colors, [0, 220, 80])
    np.testing.assert_array_equal(proposals[1].colors, [255, 140, 0])


def test_demo_applies_grasp_frame_to_tcp_to_wireframe() -> None:
    proposer = Proposer([candidate(1.0, 0.9)])
    visualizer = Visualizer()
    config = deployment_config()
    transform = np.eye(4)
    transform[0, 3] = 0.2

    run_demo(
        proposer,
        visualizer,
        gripper=config.gripper,
        grasp_frame_to_tcp=tuple(tuple(float(value) for value in row) for row in transform),  # type: ignore[arg-type]
        cloud_loader=clouds,
    )

    proposal = visualizer.layers[1].elements[0]
    assert isinstance(proposal, LineSetElement)
    np.testing.assert_allclose(proposal.vertices[1], [0.8, 0.0, 0.2], atol=1e-6)


def test_demo_rejects_non_positive_candidate_limit() -> None:
    with pytest.raises(ValueError, match="positive"):
        run_demo(
            Proposer([candidate(1.0, 0.9)]),
            Visualizer(),
            gripper=deployment_config().gripper,
            grasp_frame_to_tcp=deployment_config().grasp_frame_to_tcp,
            max_candidates=0,
            cloud_loader=clouds,
        )


def test_contributor_closes_resources_when_waiter_fails(
    mocker: MockerFixture,
) -> None:
    config = deployment_config()
    proposer = mocker.patch.object(demo_module, "GraspGenXModule").return_value
    visualizer = mocker.patch.object(demo_module, "ViserManipulationVisualizer").return_value
    mocker.patch.object(
        demo_module,
        "run_demo",
        return_value=GraspVisualizationDemoResult(3, 2, "world", "http://localhost:8095"),
    )

    with pytest.raises(RuntimeError, match="stop waiting"):
        run_contributor_demo(
            config=config,
            waiter=mocker.Mock(side_effect=RuntimeError("stop waiting")),
        )

    proposer.start.assert_called_once_with()
    proposer.stop.assert_called_once_with()
    visualizer.close.assert_called_once_with()


def test_entrypoint_passes_limit_and_handles_interrupt(
    mocker: MockerFixture, capsys: pytest.CaptureFixture[str]
) -> None:
    run = mocker.patch.object(__main__, "run_contributor_demo", side_effect=KeyboardInterrupt)

    assert __main__.main(["--max-candidates", "7"]) == 0
    run.assert_called_once_with(max_candidates=7)
    assert "stopped" in capsys.readouterr().out


def test_entrypoint_reports_optional_dependency_hint(
    mocker: MockerFixture, capsys: pytest.CaptureFixture[str]
) -> None:
    error = RuntimeError("failed to start")
    error.__cause__ = ModuleNotFoundError("graspgenx")
    mocker.patch.object(__main__, "run_contributor_demo", side_effect=error)

    assert __main__.main([]) == 1
    output = capsys.readouterr().out
    assert "uv sync --extra manipulation --extra graspgenx" in output


def test_entrypoint_rejects_non_positive_limit() -> None:
    with pytest.raises(SystemExit) as raised:
        __main__.main(["--max-candidates", "0"])

    assert raised.value.code == 2
