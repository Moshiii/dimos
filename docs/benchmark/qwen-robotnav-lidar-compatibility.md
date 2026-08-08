# LiDAR compatibility of Qwen-RobotNav benchmarks

## Conclusion

None of the indoor navigation benchmarks evaluated by Qwen-RobotNav has an
official robot-mounted LiDAR observation contract. In particular,
HM3D-OVON's released task profiles configure one RGB camera, not LiDAR or
depth. Adding a simulated LiDAR to that profile would create a useful derived
evaluation, but it would no longer reproduce the published sensor setting.

The closest faithful fit for DimOS's current navigation stack is the **2023
Habitat Challenge HM3Dv2 ObjectNav profile**, not HM3D-OVON. Its official
agent profile provides RGB-D plus GPS and compass. DimOS may deterministically
project the permitted depth image into a camera-frustum point cloud and publish
that as `PointCloud2`; this is ordinary agent-side processing of an allowed
observation. It does not, however, reproduce a rotating or 360-degree LiDAR.

If the DimOS stack strictly requires a LiDAR scan with LiDAR-specific field of
view, sampling, range, or noise, there is no faithful indoor target among the
benchmarks reported in Qwen-RobotNav. In that case we must either adapt the
stack to consume an RGB-D-derived cloud or choose a different benchmark outside
the Qwen paper.

## Simulator capability versus benchmark contract

Habitat's sensor system is configurable and supports rendered depth sensors.
Its documented agent profiles include RGB, depth, and RGB-D agents, while its
navigation task sensors include object goal, GPS, and compass observations
([Habitat configuration reference](https://github.com/facebookresearch/habitat-lab/blob/main/habitat-lab/habitat/config/CONFIG_KEYS.md)).
This makes a point cloud technically straightforward: use the depth values and
the configured camera intrinsics to back-project each valid pixel into the
camera frame, then transform it into the robot frame.

That simulator capability does not expand an official benchmark's observation
contract. The released HM3D-OVON Locobot and Stretch task configurations both
select `rgb_agent` and configure only `rgb_sensor`
([Locobot profile](https://github.com/naokiyokoyama/ovon/blob/main/config/tasks/objectnav_locobot_hm3d.yaml),
[Stretch profile](https://github.com/naokiyokoyama/ovon/blob/main/config/tasks/objectnav_stretch_hm3d.yaml)).
The reference experiment replaces the object-goal lab sensor with a CLIP goal
embedding but does not add LiDAR
([reference experiment](https://github.com/naokiyokoyama/ovon/blob/main/config/experiments/transformer_rl.yaml)).
Therefore Habitat could be customized to synthesize more range data, but doing
so would be a new sensor profile rather than the released OVON evaluation.

The Qwen-RobotNav report is consistent with those configs. It describes the
model input as one or more RGB images and reports its ObjectNav evaluation as
RGB-only. For HM3D-OVON it specifically uses one forward-facing camera
([model interface](https://arxiv.org/html/2606.18112#S2),
[ObjectNav evaluation](https://arxiv.org/html/2606.18112#S5.SS2.SSS2)).

## The RGB-D option

The 2023 Habitat Challenge ObjectNav task uses HM3D Semantics v0.2 and models a
Hello Robot Stretch with an RGB-D camera and noiseless GPS+Compass
([official challenge task](https://github.com/facebookresearch/habitat-challenge#task-objectnav)).
The corresponding Habitat configuration selects an RGB-D agent; Habitat's
official configuration reference defines the depth image and task-sensor
interfaces linked above.

Publishing an RGB-D-derived cloud as `PointCloud2` preserves that observation
contract when all of the following remain true:

- every point is derived only from the depth observation returned to the agent;
- projection uses the official resolution, field of view, depth range, pose,
  and calibration;
- the adapter does not query the scene mesh, semantic ground truth, navmesh, or
  off-camera geometry;
- invalid and out-of-range depth pixels remain invalid rather than being filled
  from privileged simulator state; and
- evaluation retains the official episodes, actions, STOP rule, and metrics.

This changes representation, not information. It is analogous to constructing
a local occupancy map from depth inside a benchmark policy. The resulting cloud
is limited to the camera frustum and depth range, so compatibility must be
verified against the exact DimOS mapping and obstacle-avoidance inputs. Calling
it simulated LiDAR would be misleading.

## Other benchmarks in the Qwen report

| Benchmark | Official observation relevant to this decision | Suitability |
| --- | --- | --- |
| VLN-CE R2R/RxR | Monocular or panoramic visual observations; no official LiDAR contract in the Qwen evaluation | No better than ObjectNav for a LiDAR-first stack |
| VLNVerse and VLN-PE | Physically embodied VLN with kinematics/physics, but Qwen's evaluation is still formulated around visual observations | Physics realism does not imply a benchmark LiDAR observation |
| EVT-Bench | Single-view active visual tracking | No |
| NAVSIM | Autonomous-driving data/evaluation can include LiDAR for some compared methods, but Qwen-RobotNav is a camera/ego-history planner and the task is road driving | Not a replacement for indoor robot navigation |

The Qwen report marks only some *other NAVSIM baselines* as using additional
LiDAR; Qwen-RobotNav itself is not marked that way
([NAVSIM table](https://arxiv.org/html/2606.18112#S5.SS4)). NAVSIM is also an
autonomous-driving planning benchmark, so adopting it would change the target
capability rather than solve the indoor sensor mismatch.

## Recommendation for the DimOS deliverable

Pivot the first faithful integration from HM3D-OVON to **HM3Dv2 ObjectNav 2023
with the official RGB-D observation profile**. The container bridge should
return RGB, depth, depth calibration, GPS, compass, and terminal benchmark
metrics. A DimOS-side adapter should convert depth to `PointCloud2` and feed the
existing mapping/navigation stack.

Before committing to the full integration, run one sensor-compatibility spike:
replay several official depth frames through the current DimOS point-cloud and
navigation modules and verify that a forward-frustum cloud is sufficient. If
those modules require 360-degree coverage or a `LaserScan`-specific geometry,
stop: adding LiDAR to Habitat would no longer be faithful to this benchmark,
and a non-Qwen benchmark with an explicit LiDAR contract should be selected.
