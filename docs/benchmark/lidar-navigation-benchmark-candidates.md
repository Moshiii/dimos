# LiDAR navigation benchmark candidates

## Decision

Use the **BARN Challenge** as the first LiDAR-native benchmark integrated with
`dimos eval`.

BARN is the best match for the existing DimOS navigation pipeline: it evaluates
the entire sense-plan-act system, standardizes a planar LiDAR-equipped robot,
accepts velocity control, publishes collision and pose truth for scoring, and
has a maintained competition with public baselines. Its limitation is equally
important: BARN explicitly evaluates **how to navigate to a supplied metric
goal**, not semantic understanding or deciding where to navigate. A DimOS LLM
agent can dispatch the navigation skill, but the meaningful score belongs to
the mapping/planning/control stack rather than to high-level agent reasoning.

Start with one public static BARN world as an end-to-end smoke case. Preserve
the official Jackal embodiment, LiDAR, start/goal, collision rule, timeout, and
score before adding DynaBARN or a larger evaluation subset.

## Candidate comparison

| Candidate | Official observation | Task and action | Official outcome | Fit |
|---|---|---|---|---|
| **BARN Challenge** | Hokuyo UST-10 planar `LaserScan`: 720 rays over 270 degrees, 0.1--30 m, 50 Hz; 1 cm range resolution and simulated Gaussian noise with 1 mm standard deviation | Jackal navigates from a fixed start to a metric goal; navigation system outputs motor/velocity commands, capped at 2 m/s | Failure on any collision or failure to reach the goal; public runner uses a 100 s timeout and 1 m goal radius. Challenge score weights collision-free success by actual traversal time versus Dijkstra-derived optimal traversal time | **Best first target**; direct low-level navigation-stack evaluation |
| **DynaBARN** | Same BARN robot interface; dynamic obstacles are added | Metric PointNav through obstacles with varied motion profiles | BARN-style navigation performance; the 2026 challenge reports DynaBARN as a bonus alongside the main score | Good second target after static BARN; dynamic behavior adds reset/reproducibility work |
| **Arena 4/5** | Robot-specific ROS 2 sensor configuration; standard planner integrations use `sensor_msgs/msg/LaserScan`, but Arena does not define one immutable LiDAR geometry across every robot/simulator | Continuous or discrete velocity actions; point-goal, waypoint, exploration, and social-navigation scenarios | Evaluation pipeline includes success, collisions, time, path length, kinematics, and social metrics | Richest follow-up for dynamic/social navigation, but less canonical as one frozen benchmark and much harder to integrate faithfully |
| **iGibson PointNav** | Configurable 1-beam scan defaults to 228 rays, 240 degrees, 0.05--5.6 m; ROS integration can publish the scan as `sensor_msgs/PointCloud2` | Gym-style continuous linear/angular velocity PointNav; generic task defaults include 500 steps and 0.36 m goal tolerance | Configurable reward/termination components | Useful simulator capability, **not a LiDAR benchmark contract**. The official 2021 Interactive/Social Navigation challenge instead required goal, velocity, and RGB-D observations |
| **BenchBot/BEAR** | RGB-D, pose, and flatscan LiDAR are official observations | Discrete robot motion while actively observing an indoor scene | Semantic-map quality for Semantic SLAM and Scene Change Detection | Agentic and LiDAR-equipped, but it benchmarks active scene understanding rather than navigation success |

## BARN contract to preserve

### Sensor and embodiment

The current challenge standardizes a Clearpath Jackal with 2D LiDAR and a
2 m/s maximum speed. The official runner selects the `ust10` model and consumes
ROS `sensor_msgs/LaserScan` on `front/scan`. The pinned Jackal description used
by that runner specifies:

- 720 horizontal samples from -2.35619 to +2.35619 radians (270 degrees);
- 50 Hz update rate;
- 0.1 m minimum and 30 m maximum range;
- 0.01 m range resolution; and
- zero-mean Gaussian range noise with 0.001 m standard deviation.

Sources: [2026 BARN rules](https://people.cs.gmu.edu/~xiao/Research/BARN_Challenge/BARN_Challenge26.html),
[official runner sensor selection](https://github.com/Daffan/the-barn-challenge/blob/bf5a226f6088ec96bf0d2dbee3253a8ea6119b83/run.py#L40-L43),
[pinned Jackal UST-10 simulation description](https://github.com/jackal/jackal/blob/0d8d76f96bd52102b69a3b9cb735fd5f9e15f695/jackal_description/urdf/accessories/hokuyo_ust10.urdf.xacro#L3-L46).

DimOS currently consumes `PointCloud2`, not `LaserScan`. A lossless planar
adapter can project each valid polar return into `(x, y, z=0)` in the LiDAR
frame. That preserves the official observation; querying the Gazebo world,
occupancy map, or planned path would not.

### Episode and scoring

The public dataset has 300 generated environments and includes Gazebo worlds,
occupancy/C-space grids, Dijkstra paths, and ROS map files. The competition
uses 50 held-out generated environments and averages ten trials per world. The
official task is to reach the goal without any collision. Its per-world score
is:

```text
success * OT / clip(AT, 2 * OT, 8 * OT)
```

where `AT` is actual traversal time and `OT` is the provided shortest path
length divided by the 2 m/s maximum speed. The current public runner fixes the
static-world start pose at `(-2.25, 3, pi/2)`, places the goal 10 m ahead, stops
on collision, considers the goal reached within 1 m, and times out at 100 s.

Sources: [BARN dataset and scope](https://www.cs.utexas.edu/~xiao/BARN/BARN.html),
[challenge scoring](https://people.cs.gmu.edu/~xiao/Research/BARN_Challenge/BARN_Challenge26.html),
[official runner termination](https://github.com/Daffan/the-barn-challenge/blob/bf5a226f6088ec96bf0d2dbee3253a8ea6119b83/run.py#L140-L169).

### Runtime and access

The official runner targets Ubuntu 18.04, ROS Melodic, and Gazebo, and provides
a Singularity definition bootstrapped from the `ros:melodic` Docker image. It
is therefore a natural pinned OCI/container runtime for `dimos eval`; DimOS
should communicate through a narrow bridge instead of importing ROS 1. The
runner repository is MIT licensed. The dataset is publicly downloadable, but
its webpage does not state a separate dataset license, so redistribution terms
should be confirmed before baking it into an image.

Sources: [official runner and container instructions](https://github.com/Daffan/the-barn-challenge),
[runner license](https://github.com/Daffan/the-barn-challenge/blob/bf5a226f6088ec96bf0d2dbee3253a8ea6119b83/LICENSE),
[dataset download](https://www.cs.utexas.edu/~xiao/BARN/BARN.html).

## Why not iGibson first

iGibson is attractive because its generic environment supports planar LiDAR
and its ROS adapter already publishes `PointCloud2`. However, those are
simulator features rather than the fixed sensor contract of its renowned
navigation challenge. The official 2021 Interactive and Social Navigation
tasks exposed relative goal, current velocities, and RGB-D, with normalized
linear/angular velocity actions. Adding the generic scan would create an
iGibson-derived task, not a comparable challenge result.

Sources: [iGibson sensor and task configuration](https://stanfordvl.github.io/iGibson/environments.html),
[iGibson ROS topics](https://stanfordvl.github.io/iGibson/ros_integration.html),
[2021 challenge setup](https://svl.stanford.edu/igibson/challenge2021.html).

## Why Arena is second, not first

Arena is active, ROS 2-native, supports Gazebo/Unity/Isaac/Flatland, and has
point-goal, waypoint, exploration, and social-navigation modes. This makes it
the better later target for evaluating reactive behavior around people. But
the robot configuration selects the sensor sources and action ranges, so
"Arena with LiDAR" is a configuration family rather than one frozen sensor
and episode protocol. Integrating it first would require choosing and then
maintaining our own benchmark profile before establishing the basic
container-to-DimOS navigation seam.

Sources: [Arena project](https://github.com/Arena-Rosnav),
[robot sensor/action configuration](https://arena-rosnav.readthedocs.io/en/latest/tutorials/simulation/robot/),
[task modes](https://arena-rosnav.readthedocs.io/en/latest/tutorials/tasks/),
[benchmark runner](https://arena-rosnav.readthedocs.io/en/latest/tutorials/tasks/benchmark/).

## First `dimos eval` deliverable

The first deliverable should prove one official public BARN episode end to end:

1. `dimos eval` starts a pinned BARN ROS Melodic/Gazebo container.
2. The bridge emits the official `LaserScan`, odometry/pose, runtime state, and
   collision/terminal events, and accepts velocity commands.
3. DimOS converts the planar scan to `PointCloud2` and runs its normal
   voxel/costmap/planner/controller path.
4. The DimOS agent receives a metric navigation task and invokes the normal
   navigation skill. The LLM does not receive the BARN occupancy map or path.
5. The container remains authoritative for collision, goal reach, timeout, and
   timing. The official BARN runner computes the result and score; `dimos eval`
   stores those benchmark-emitted values verbatim alongside the world ID, image
   digest, actions, and trajectory. DimOS does not define additional success
   gates, recompute an authoritative score, or blend BARN metrics with its own.

After a scripted integration smoke, run the scored case through the DimOS agent
system. The smoke validates only bridge behavior and artifact capture; it does
not produce an alternative benchmark judgment. The benchmark-emitted result is
honest evidence for the DimOS navigation stack, but should not be presented as
a benchmark of LLM reasoning.
