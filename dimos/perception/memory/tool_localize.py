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

"""Query memory for an object, localize it in 3D via depth, and render in Rerun.

Run: uv run python -m dimos.perception.memory.tool_localize [query] [out.rrd]
         [--from <s>] [--duration <s>]
"""

from __future__ import annotations

import argparse
from pathlib import Path

from dimos.memory2.store.sqlite import SqliteStore
from dimos.memory2.transform import throttle
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.perception.detection.type.detection3d.pointcloud import Detection3DPC
from dimos.perception.memory.prompted_object_localizer import (
    DEFAULT_OPTICAL_FRAME,
    PromptedObjectLocalizationRuntime,
)
from dimos.utils.data import get_data
from dimos.visualization.rerun.init import rerun_init


def write_object_cloud(path: Path, cloud: PointCloud2) -> None:
    """Write a segmented object cloud without eagerly importing Open3D."""
    import open3d as o3d  # type: ignore[import-untyped]

    path.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(path), cloud.pointcloud):
        raise RuntimeError(f"failed to write object point cloud: {path}")


def _render(
    runtime: PromptedObjectLocalizationRuntime,
    best: Detection3DPC,
    out: Path,
) -> None:
    """Write the debug Rerun recording from localization runtime streams."""
    import rerun as rr
    import rerun.blueprint as rrb

    rerun_init("memory-localize")
    rr.save(out)
    rr.send_blueprint(
        rrb.Blueprint(
            rrb.Horizontal(
                rrb.Spatial3DView(origin="/", name="Scene"),
                rrb.Spatial2DView(origin="camera", name="Live"),
                column_shares=[2, 1],
            )
        )
    )

    green, red = [46, 204, 113], [231, 76, 60]
    point_size = 0.005

    def at(timestamp: float) -> None:
        rr.set_time("ts", timestamp=timestamp)

    backdrop_observation = runtime.detections_3d.first()
    backdrop = PointCloud2.from_rgbd(
        backdrop_observation.data.image,
        runtime.depth_at(backdrop_observation),
        runtime.camera_info,
        depth_scale=0.001,
    ).transform(-runtime.world_to_optical(backdrop_observation.ts))
    rr.log("map", backdrop.voxel_downsample(0.01).to_rerun(voxel_size=point_size), static=True)

    rr.log("camera", runtime.camera_info.to_rerun(), static=True)
    for observation in runtime.images.transform(throttle(0.1)):
        at(observation.ts)
        rr.log("camera/image", observation.data.to_rerun())
        rr.log("camera", runtime.camera_pose(observation.ts).to_rerun())

    for index, observation in enumerate(runtime.detections):
        at(observation.ts)
        annotated = observation.data.annotated_image()
        rr.log("camera/image", annotated.to_rerun())
        frame = f"detections/frames/{index}"
        rr.log(frame, runtime.camera_pose(observation.ts).to_rerun())
        rr.log(frame, runtime.camera_info.to_rerun())
        rr.log(f"{frame}/image", annotated.to_rerun())

    for tag, stream, color in (
        ("matched", runtime.detections_3d, green),
        ("verified", runtime.verified_3d, red),
    ):
        for observation_index, observation in enumerate(stream):
            at(observation.ts)
            for detection_index, detection in enumerate(observation.data):
                rr.log(
                    f"detections/{tag}/{observation_index}_{detection_index}_"
                    f"{detection.name.replace(' ', '_')}",
                    detection.pointcloud.to_rerun(voxel_size=point_size, colors=color),
                )

    at(best.ts)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("query", nargs="?", default="plant")
    parser.add_argument("out", nargs="?", type=Path, default=Path("localize.rrd"))
    parser.add_argument("--dataset", type=Path, help="Memory2 recording database")
    parser.add_argument(
        "--from",
        dest="start",
        type=float,
        default=0.0,
        help="start offset into the recording (s)",
    )
    parser.add_argument("--duration", type=float, default=None, help="how much to parse (s)")
    parser.add_argument(
        "--object-cloud-out",
        type=Path,
        help="write the strongest segmented 3D detection as a PLY or PCD point cloud",
    )
    return parser


def main(argv: list[str] | None = None) -> int:
    """Run the standalone prompted-localization debug workflow."""
    args = _parser().parse_args(argv)
    dataset = args.dataset or get_data(
        "xarm6_worldbelief_realsense_d435i_stationery_calibrated/"
        "xarm6_worldbelief_20260729_203624_161992.db"
    )

    with SqliteStore(path=dataset, must_exist=True) as store:
        lo, hi = store.streams.color_image.get_time_range()
        start = lo + args.start
        end = min(hi, start + args.duration) if args.duration is not None else hi
        span = end - start
        print(
            f"window: {args.start:.0f}s → {args.start + span:.0f}s of the recording ({span:.0f}s)"
        )

        with PromptedObjectLocalizationRuntime(
            store,
            optical_frame=DEFAULT_OPTICAL_FRAME,
            report=print,
        ) as runtime:
            best = runtime.localize(args.query, start, end)
            if best is None:
                print(f"no usable 3D detection found for '{args.query}'")
                return 1
            if args.object_cloud_out is not None:
                write_object_cloud(args.object_cloud_out, best.pointcloud)
                print(f"saved segmented object cloud to {args.object_cloud_out}")
            runtime.verify_cross_view(best, start=start, end=end)
            _render(runtime, best, args.out)

    print(f"saved {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
