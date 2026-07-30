(doc-platforms-quadruped-go2-index-unitree-go2)=

# Unitree Go2

- {doc}`Setup your Dog </platforms/quadruped/go2/setup>` — requirements, install, connecting to your Go2, and agentic control
- {doc}`Simulation </platforms/quadruped/go2/simulation>` — try it with no hardware via replay or MuJoCo
- {doc}`Mapping & Navigation </capabilities/navigation/index>` — live nav, premap recording, and relocalization

(doc-platforms-quadruped-go2-index-available-blueprints)=

## Available Blueprints

| Blueprint                              | Description                                                |
| -------------------------------------- | ---------------------------------------------------------- |
| `dimos run unitree-go2-basic`          | Connection + visualization (no navigation)                 |
| `dimos run unitree-go2`                | Full navigation stack                                      |
| `dimos run unitree-go2-agentic`        | Navigation + LLM agent + MCP tool access                   |
| `dimos run unitree-go2-agentic-ollama` | Agent with local Ollama models                             |
| `dimos run unitree-go2-spatial`        | Navigation + spatial memory                                |
| `dimos run unitree-go2-detection`      | Navigation + object detection                              |
| `dimos run unitree-go2-memory`         | Navigation + record `lidar`/`odom`/`color_image` to `.db`  |
| `dimos run unitree-go2-relocalization` | Navigation + align live scans to a saved `.pc2.lcm` premap |

(doc-platforms-quadruped-go2-index-deep-dive)=

## Deep Dive

- {doc}`Navigation overview </capabilities/navigation/index>` — live mapping vs premap relocalization
- {doc}`Navigation stack </capabilities/navigation/deep_dive>` — column-carving voxel mapping, costmap generation, A\* planning
- {doc}`Relocalization </capabilities/navigation/relocalization>` — record → `dimos map global --export` → replay or live deploy
- {doc}`Visualization </usage/visualization>` — Rerun, performance tuning
- {doc}`Data Streams </usage/data_streams/index>` — RxPY streams, backpressure, quality filtering
- {doc}`Transports </usage/transports/index>` — LCM, SHM, DDS
- {doc}`Blueprints </usage/blueprints>` — composing modules

```{toctree}
:hidden: true
:maxdepth: 1

setup
simulation
```
