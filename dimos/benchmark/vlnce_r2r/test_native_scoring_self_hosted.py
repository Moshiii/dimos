# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Real-container native scoring gates driven only through the public protocol."""

from collections.abc import Iterator, Sequence
import heapq
from pathlib import Path
from queue import Queue
import subprocess
import time
from typing import Any, Literal
import uuid

import cv2
import grpc  # type: ignore[import-untyped]
import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt
from scipy.spatial.transform import Rotation

from dimos.benchmark.agent_eval.case import (
    BenchmarkInstructionTask,
    EvalCase,
    ExternalBenchmarkEpisodeSource,
)
from dimos.benchmark.vlnce_r2r.external_engine import RESULT_SCHEMA_PATH
from dimos.benchmark.vlnce_r2r.external_runtime import (
    _IDENTITY_FIELDS,
    VlnceExternalRuntime,
    _write_json,
)
from dimos.benchmark.vlnce_r2r.native_result import VlnceNativeResult, validate_native_result
from dimos.benchmark.vlnce_r2r.preparation import (
    PreparationReceipt,
    prepare_public_assets,
    resolve_oci_image,
)
from dimos.benchmark.vlnce_r2r.protocol import vlnce_public_v1_pb2 as pb
from dimos.benchmark.vlnce_r2r.protocol.contract import expected_handshake
from dimos.benchmark.vlnce_r2r.protocol.vlnce_public_v1_pb2_grpc import (
    VlncePublicGatewayStub,
)

CONTROL_PERIOD_SECONDS = 0.1
STEP_METRES = 0.04


@pytest.mark.self_hosted_large
def test_public_controller_scores_far_failure_and_reference_route_success(
    tmp_path: Path,
) -> None:
    case_path = Path(__file__).parent / "cases/mp3d-example-episode-515/case.json"
    case = EvalCase.model_validate_json(case_path.read_bytes())
    source = case.source
    task = case.task
    assert isinstance(source, ExternalBenchmarkEpisodeSource)
    assert isinstance(task, BenchmarkInstructionTask)
    preparation = prepare_public_assets(source, task)
    image_id = resolve_oci_image(source.preparation.image)
    reference_path = preparation.episode.episode["reference_path"]

    far, far_initial, far_steps, far_render = _run_public_attempt(
        case,
        preparation,
        image_id,
        tmp_path / "far",
        reference_path=None,
        render="native",
    )
    timed_out, timeout_initial, timeout_steps, timeout_render = _run_public_attempt(
        case,
        preparation,
        image_id,
        tmp_path / "timeout",
        reference_path=None,
        timeout_seconds=1.0,
    )
    near, near_initial, near_steps, near_render = _run_public_attempt(
        case, preparation, image_id, tmp_path / "near", reference_path=reference_path
    )

    expected_start = preparation.episode.episode["start_position"]
    assert far_initial == pytest.approx(expected_start, abs=1e-4)
    assert near_initial == pytest.approx(expected_start, abs=1e-4)
    assert timeout_initial == pytest.approx(expected_start, abs=1e-4)
    assert far_steps == 0
    assert timeout_steps == 0
    assert far.metrics.SUCCESS == 0.0
    assert far.metrics.DISTANCE_TO_GOAL > 3.0
    assert timed_out.terminal_reason == "timeout"
    assert timed_out.metrics.SUCCESS == 0.0
    assert near_steps > 0
    assert near.metrics.SUCCESS == 1.0
    assert near.metrics.DISTANCE_TO_GOAL < 3.0
    assert near.metrics.STEPS_TAKEN == near_steps + 1
    assert far_render is not None
    assert far_render["status"] == "completed"
    assert far_render["frame_count"] == 20
    assert timeout_render is None
    assert near_render is None
    capture = cv2.VideoCapture(str(tmp_path / "far/native-render.mp4"))
    try:
        ok, frame = capture.read()
        assert ok
        assert frame.shape == (448, 896, 3)
    finally:
        capture.release()

    broken_scorer = subprocess.run(
        [
            "podman",
            "run",
            "--rm",
            "--entrypoint",
            "python",
            image_id,
            "-c",
            (
                "from vlnce_runtime.result import build_result; "
                "build_result({}, 'submitted', [[0, 0, 0]], {}, {}, 1.0)"
            ),
        ],
        check=False,
        capture_output=True,
        text=True,
    )
    assert broken_scorer.returncode != 0
    assert "official metric set" in broken_scorer.stderr


def _run_public_attempt(
    case: EvalCase,
    preparation: PreparationReceipt,
    image_id: str,
    attempt_path: Path,
    reference_path: Sequence[Sequence[float]] | None,
    timeout_seconds: float | None = None,
    render: Literal["none", "native"] = "none",
) -> tuple[VlnceNativeResult, list[float], int, dict[str, Any] | None]:
    attempt_id = f"attempt_{uuid.uuid4().hex}"
    runtime = VlnceExternalRuntime(
        case=case,
        attempt_id=attempt_id,
        attempt_path=attempt_path,
        preparation=preparation,
        image_id=image_id,
        render=render,
    )
    runtime._make_directories()
    private_case = runtime.private_case()
    if timeout_seconds is not None:
        private_case["timeout_seconds"] = timeout_seconds
    _write_json(runtime.private_dir / "private-case.json", private_case)
    _write_json(
        runtime.private_dir / "expected-identity.json",
        {field: private_case[field] for field in _IDENTITY_FIELDS},
    )
    cdi_global, cdi_run = runtime._prepare_cdi()
    runtime._log_handle = runtime.log_path.open("xb")
    runtime._process = subprocess.Popen(
        runtime._container_command(cdi_global, cdi_run),
        stdin=subprocess.DEVNULL,
        stdout=runtime._log_handle,
        stderr=subprocess.STDOUT,
    )
    try:
        _wait_for_socket(runtime)
        requests: Queue[pb.ClientMessage | None] = Queue()

        def request_iterator() -> Iterator[pb.ClientMessage]:
            while True:
                request = requests.get()
                if request is None:
                    return
                yield request

        channel = grpc.insecure_channel(f"unix://{runtime.socket_path}")
        responses = VlncePublicGatewayStub(channel).Stream(request_iterator())
        public_identity = {key: str(value) for key, value in private_case.items()}
        try:
            requests.put(pb.ClientMessage(handshake=expected_handshake(public_identity)))
            assert next(responses).HasField("ready")
            requests.put(
                pb.ClientMessage(lifecycle=pb.LifecycleCommand(kind=pb.LifecycleCommand.BEGIN))
            )
            assert next(responses).HasField("acknowledgement")
            observation = next(responses).observation
            initial_position = _position(observation).tolist()
            steps = 0
            if reference_path is not None:
                target = np.asarray(reference_path[-1], dtype=np.float64)
                for waypoint in _public_map_path(
                    observation.static_map, _position(observation), target
                ):
                    observation, target_steps = _drive_to(
                        requests, responses, observation, waypoint, steps
                    )
                    steps += target_steps
            if timeout_seconds is None:
                requests.put(
                    pb.ClientMessage(
                        submit_route=pb.SubmitRoute(
                            command_sequence=steps + 1,
                            observation_sequence=observation.sequence,
                        )
                    )
                )
                acknowledgement = next(responses).acknowledgement
                assert acknowledgement.kind == pb.Acknowledgement.ROUTE_SUBMITTED
            payload = _wait_for_result(runtime)
        finally:
            requests.put(None)
            responses.cancel()
            channel.close()
        result = validate_native_result(
            payload,
            case=case,
            attempt_id=attempt_id,
            schema_path=RESULT_SCHEMA_PATH,
        )
        assert list(runtime.result_dir.iterdir()) == [runtime.result_path]
    finally:
        runtime.close()
    return result, initial_position, steps, runtime.render_evidence()


def _drive_to(
    requests: Queue[pb.ClientMessage | None],
    responses: Any,
    observation: pb.Observation,
    target: np.ndarray[Any, np.dtype[np.float64]],
    prior_steps: int,
) -> tuple[pb.Observation, int]:
    steps = 0
    while np.linalg.norm((_position(observation) - target)[[0, 2]]) > 0.04:
        if steps >= 20:
            raise AssertionError(
                "public controller did not reach adjacent map cell "
                f"{target.tolist()} from {_position(observation).tolist()}"
            )
        position = _position(observation)
        world_delta = target - position
        world_delta[1] = 0.0
        distance = float(np.linalg.norm(world_delta))
        world_delta *= min(STEP_METRES, distance) / distance
        pose = observation.world_from_base
        local_delta = (
            Rotation.from_quat([pose.qx, pose.qy, pose.qz, pose.qw]).inv().apply(world_delta)
        )
        command_sequence = prior_steps + steps + 1
        requests.put(
            pb.ClientMessage(
                control=pb.PlanarControl(
                    command_sequence=command_sequence,
                    observation_sequence=observation.sequence,
                    linear_x=-local_delta[2] / CONTROL_PERIOD_SECONDS,
                    linear_y=-local_delta[0] / CONTROL_PERIOD_SECONDS,
                )
            )
        )
        assert next(responses).acknowledgement.command_sequence == command_sequence
        observation = next(responses).observation
        steps += 1
    return observation, steps


def _position(observation: pb.Observation) -> np.ndarray:
    pose = observation.world_from_base
    return np.array([pose.x, pose.y, pose.z], dtype=np.float64)


def _public_map_path(
    occupancy: pb.OccupancyMap,
    start: np.ndarray[Any, np.dtype[np.float64]],
    target: np.ndarray[Any, np.dtype[np.float64]],
) -> list[np.ndarray[Any, np.dtype[np.float64]]]:
    traversable = np.frombuffer(occupancy.traversability, dtype=np.uint8).reshape(
        occupancy.height, occupancy.width
    )
    safe = traversable == 1
    clearance = distance_transform_edt(safe)

    def nearest_cell(
        position: np.ndarray[Any, np.dtype[np.float64]], candidates_mask: np.ndarray[Any, Any]
    ) -> tuple[int, int]:
        requested = np.array(
            [
                round((position[2] - occupancy.origin.z) / occupancy.resolution),
                round((position[0] - occupancy.origin.x) / occupancy.resolution),
            ],
            dtype=np.int64,
        )
        candidates = np.argwhere(candidates_mask)
        if candidates.size == 0:
            raise AssertionError("public occupancy map has no cylinder-safe cells")
        nearest = candidates[np.argmin(np.sum((candidates - requested) ** 2, axis=1))]
        return int(nearest[0]), int(nearest[1])

    start_cell = nearest_cell(start, safe)
    preferred_goal = clearance >= max(2.0, 0.1 / occupancy.resolution)
    target_cell = nearest_cell(target, preferred_goal)
    frontier: list[tuple[float, tuple[int, int]]] = [(0.0, start_cell)]
    cost = {start_cell: 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == target_cell:
            break
        for row_delta, column_delta in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            neighbour = (current[0] + row_delta, current[1] + column_delta)
            if not (
                0 <= neighbour[0] < safe.shape[0]
                and 0 <= neighbour[1] < safe.shape[1]
                and safe[neighbour]
            ):
                continue
            new_cost = cost[current] + 1.0 + 4.0 / (clearance[neighbour] + 0.25)
            if new_cost >= cost.get(neighbour, float("inf")):
                continue
            cost[neighbour] = new_cost
            parent[neighbour] = current
            heuristic = abs(neighbour[0] - target_cell[0]) + abs(neighbour[1] - target_cell[1])
            heapq.heappush(frontier, (new_cost + heuristic, neighbour))
    if target_cell not in cost:
        raise AssertionError("public occupancy map has no path to the test endpoint")
    cells = [target_cell]
    while cells[-1] != start_cell:
        cells.append(parent[cells[-1]])
    cells.reverse()
    return [
        np.array(
            [
                occupancy.origin.x + column * occupancy.resolution,
                start[1],
                occupancy.origin.z + row * occupancy.resolution,
            ],
            dtype=np.float64,
        )
        for row, column in cells[1:]
    ]


def _wait_for_socket(runtime: VlnceExternalRuntime) -> None:
    deadline = time.monotonic() + 60.0
    while not runtime.socket_path.exists():
        runtime._raise_if_container_exited("before the public scorer test")
        if time.monotonic() >= deadline:
            raise TimeoutError("real container did not publish its public socket")
        time.sleep(0.05)


def _wait_for_result(runtime: VlnceExternalRuntime) -> bytes:
    deadline = time.monotonic() + 30.0
    while not runtime.result_path.is_file():
        runtime._raise_if_container_exited("before publishing its native result")
        if time.monotonic() >= deadline:
            raise TimeoutError("real container did not publish its native result")
        time.sleep(0.05)
    assert runtime._process is not None
    assert runtime._process.wait(timeout=10.0) == 0
    return runtime.result_path.read_bytes()
