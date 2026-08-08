# DimOS agent evaluation stack: a conceptual overview

This note explains how we evaluate agents against recorded, simulated, and real
robot experiences. It is written for readers who do not need to know the DimOS
codebase.

## The short version

DimOS connects an AI agent to a robot's observations and capabilities. The
evaluation stack gives the agent a controlled situation, asks it to complete a
task, and checks the result without showing it the expected answer.

Today, the simplest evaluation uses a frozen robot recording:

```text
past robot run
     |
     v
frozen Memory2 snapshot ---> agent investigates ---> agent answers
                                                        |
                                                        v
                                             private validator checks it
```

The same structure will support long-running simulations and real robots. In
that setting, memory updates continuously, the agent can act through DimOS, and
the validator observes what happens over time.

## The main concepts

### DimOS

DimOS is the software layer that joins robot hardware, perception, mapping,
memory, planning, and agent-controlled actions. A DimOS **module** is one
component, such as a camera processor or mapper. A **blueprint** combines the
modules needed for one runnable robot system.

```text
                     DimOS blueprint
       +-----------------------------------------+
       |                                         |
camera ---> perception ---> Memory2 ---> mapping |
       |                              \          |
       |                               navigation ---> robot
       +-----------------------------------------+
```

An evaluation does not need to expose these internal components directly. It
gives the agent a small, stable interface for understanding and controlling the
system.

### Memory2

Memory2 is the robot's timestamped experience store. It records the message
streams produced during a run, including raw observations and derived results.
Depending on the robot configuration, this can include images, poses, detected
objects, maps, and other perception outputs.

It is useful to think of Memory2 as a searchable timeline rather than a chat
history:

```text
time --------------------------------------------------------------->

camera     [image]----[image]----[image]----[image]----[image]
pose       [pose]--[pose]--[pose]--[pose]--[pose]--[pose]--[pose]
objects          [chair]----------[door]--------------[table]
map              [map 1]----------[map 2]--------------[map 3]
                                      ^
                                      |
                              selected evaluation point
```

For a frozen evaluation, the agent sees only information at or before the
selected point. A normalized progress value makes the selection portable:

```text
0.0                  0.5                  1.0
start --------------- middle --------------- end
```

For example, progress `1.0` means the complete recording.

For a live evaluation, Memory2 keeps receiving new messages while the agent is
working.

### CodePolicy

CodePolicy gives the agent one persistent Python workspace and one tool,
`python_exec`. The agent can write small Python queries to inspect Memory2,
combine observations, count items, or calculate summaries. Variables remain
available across calls within the same evaluation attempt.

```text
agent idea
   |
   | "inspect the available streams"
   v
python_exec("memory...query...")
   |
   v
result returned to agent
   |
   v
agent decides the next query
```

CodePolicy is a trusted, unsandboxed execution environment. It is a flexible
research interface, not a security boundary.

### Pi

Pi is the agent loop. It receives the task, decides what to investigate, calls
`python_exec`, reads the results, and eventually produces an answer. Pi is the
reasoning and tool-use layer; CodePolicy is the workspace it operates.

### Porcelain

Porcelain is the programmatic interface to a running DimOS system. It lets an
agent inspect modules and call robot capabilities without becoming part of the
robot blueprint itself.

In a frozen evaluation, the agent gets `memory` but no `app`. In a live
simulation or real-robot evaluation, it gets both:

```text
frozen recording                 live simulation or robot
----------------                 ------------------------
memory = frozen Memory2          memory = live Memory2
app    = unavailable             app    = DimOS Porcelain
```

## The common evaluation shape

Every evaluation separates four questions:

```text
+---------------+     +---------------+
|    SOURCE     |     |     TASK      |
|               |     |               |
| What world or |     | What should   |
| evidence is   |     | the agent do? |
| available?    |     |               |
+-------+-------+     +-------+-------+
        |                     |
        +----------+----------+
                   |
                   v
          +--------+---------+
          |   INTERACTION    |
          |                  |
          | How does the     |
          | agent receive    |
          | data and act?    |
          +--------+---------+
                   |
                   v
          +--------+---------+
          |    VALIDATOR     |
          |                  |
          | How do we check  |
          | success?         |
          +------------------+
```

- **Source:** a frozen recording, simulator scene, or live robot environment.
- **Task:** a question or goal given to the agent.
- **Interaction:** the tools, data, timing, and lifecycle the agent receives.
- **Validator:** the private procedure that decides whether the result succeeds.

This separation prevents each benchmark from inventing a new runner. A new
evaluation usually adds a source, interaction, or validator while reusing the
same attempt and evidence machinery.

### How case configuration differs

Frozen QA and real-time simulation use the same four-part case shape, but bind
different facts:

| Contract | Frozen QA | Real-time simulator | External native benchmark |
| --- | --- | --- | --- |
| Source | recording and progress | scene, robot, and preparation | episode, scene assets, OCI image, and protocol |
| Task | question | embodied instruction | exact upstream route instruction |
| Interaction | bounded frozen CodePolicy turn | live CodePolicy with timeout | live CodePolicy until native STOP or timeout |
| Validator | private expected answer | private periodic goal check | benchmark-owned native result |

The external benchmark keeps its oracle, reference path, progress, and scoring
inside its container. DimOS receives RGB, depth, pose, a geometry-only map, and
bounded planar control over a public Unix-domain socket. The evaluator alone
reads the terminal native result.

```text
private benchmark container              public DimOS runtime
+--------------------------------+       +---------------------------+
| episode + reference path       |       | connection + navigation   |
| Habitat simulation             |<----->| memory + CodePolicy + Pi  |
| official VLN-CE measures       |  UDS  | submit_route()            |
+---------------+----------------+       +---------------------------+
                |
                v
       evaluator-only result
```

## Short-horizon frozen QA

A frozen question-answering attempt works like this:

```text
1. Load an immutable case
             |
             v
2. Verify recording, prepared data, and private validator
             |
             v
3. Open Memory2 at the selected recording progress
             |
             v
4. Start a fresh standalone CodePolicy workspace
             |
             v
5. Start a fresh Pi session and provide the public question
             |
             v
6. Pi investigates through repeated python_exec calls
             |
             v
7. Pi returns: explanation + "ANSWER: <integer>"
             |
             v
8. Parse the answer and check it with the private validator
             |
             v
9. Save evidence and stop all owned processes
```

The agent can access:

```text
public question + read-only frozen memory + python_exec
```

The agent cannot access:

```text
private expected answer + validator notes + live robot controls
```

The CLI shows the public session context before the agent starts:

```text
[eval] Session
  Case       go2-hongkong-office-room-count-smoke
  Source     go2_hongkong_office @ 100%
  Question   How many rooms in total?
  Answer     pending
```

It then streams visible agent text and bounded CodePolicy calls. Hidden model
reasoning, credentials, and private validator data are not printed.

### Hong Kong office example

The current concrete case uses the complete Hong Kong office recording:

```text
go2_hongkong_office recording
              |
              | progress = 1.0
              v
complete frozen Memory2 view
              |
              | read-only memory API
              v
Pi + CodePolicy: "How many rooms in total?"
              |
              v
typed integer prediction
              |
              v
private exact-integer validator
```

Run the smoke case with:

```bash
uv run dimos eval run \
  dimos/benchmark/short_horizon_qa/cases/go2_hongkong_office-room-count-smoke/case.json \
  --output=/tmp/dimos-eval-smoke
```

The smoke case currently uses a synthetic expected value to test the plumbing.
It is not an authoritative room-count benchmark. The real acceptance case still
needs a human-authored room inventory, a clear counting policy, and independent
review. We must not derive the expected answer from the agent being evaluated.

## Real-time simulator evaluation

A long-horizon agent must observe, act, and adjust over time. It runs
beside a live simulator or real DimOS robot system.

```text
              +---------------------------------------+
              |        live DimOS system              |
              |                                       |
world/robot ---> sensors ---> perception ---> Memory2 |
     ^        |                                |      |
     |        +--------------------------------|------+
     |                                         |
     |                            new observations
     |                                         |
     |                                         v
     |                              +----------+---------+
     |                              | Pi + CodePolicy    |
     |                              |                   |
     |                              | memory: observe   |
     |                              | app: act through  |
     |                              |      Porcelain    |
     |                              +----------+---------+
     |                                         |
     |                                    robot action
     +-----------------------------------------+
```

The validator runs alongside this loop:

```text
                    task begins
                         |
                         v
observe ---> reason ---> act ---> observe ---> reason ---> act
   |                                                    |
   +---------------- validator observes ---------------+
                         |
                         v
             success, failure, or timeout
```

Examples include:

- find an object and report its location;
- navigate through several rooms without collision;
- inspect a scene, manipulate an object, and verify the final state;
- remember an earlier observation and use it later in the task;
- recover when an action fails or the world changes.

The source is now a live environment rather than a sealed recording. The task
may span many turns. The interaction gives the agent both live `memory` and the
Porcelain-backed `app`. The validator may watch robot state throughout the run
instead of checking only one final answer.

The static and real-time cases vary their contracts, not their command:

| Contract | Frozen QA | Real-time simulation |
|---|---|---|
| `source` | recording + progress | scene + provider + robot + DimOS blueprint + optional preparation |
| `task` | integer question | embodied instruction |
| `interaction` | frozen CodePolicy | live CodePolicy + case deadline |
| `validator` | private exact integer | private periodically sampled world-state goal |
| agent interface | read-only frozen `memory` | updating read-only `memory` + Porcelain `app` |
| success | parsed answer matches | evaluator observes the private goal |

The evaluator owns the live runtime. It loads the case-bound simulator provider,
starts a non-agentic DimOS blueprint, waits for sensors, odometry, Memory2,
Porcelain, and motion control, runs source preparation, and only then starts the
task deadline and external Pi session. The control thread periodically checks
the private goal while Pi is active. Pi text and `/goal_reached` are never
evaluation truth. Goal success ends the attempt immediately; at the deadline,
the evaluator samples once more before recording `episode_timeout`.

### PiMSim Go2 apartment example

The checked-in case reproduces the apartment go-to-bed system test. PiMSim
materializes the `dimsim-apartment` scene. Before Pi starts, the evaluator drives
Go2 through the same fixed public exploration route with `/cmd_vel` to populate
spatial memory. It then performs the original single respawn at
`(3.0, 0.0, 0.52)` and verifies odometry convergence. Pi receives exactly
`Go to the bed`. A private validator queries semantic bed bounds and checks the
robot's current planar distance periodically. The physical exploration can be
slower or somewhat flaky, but it preserves the observations and dynamics of the
original end-to-end setup instead of teleporting between route points.

Install the PiMSim core and DimOS integration packages (editable checkouts are
fine), build `packages/pi-spatial-adapter`, and configure either Codex OAuth in
`~/.pi/agent/auth.json` or `OPENAI_API_KEY`. Then run:

```bash
uv run dimos eval run \
  dimos/benchmark/realtime_sim/cases/go2-apartment-go-to-bed/case.json \
  --output=/tmp/dimos-eval-go-to-bed
```

This command executes one case and one attempt. The apartment exploration
route is source preparation for that attempt, not a collection of additional
tests, and `dimos eval run` does not invoke the self-hosted pytest suite.

To observe the same case through Rerun Web, pass the global visualization
options before `eval`:

```bash
uv run dimos \
  --viewer rerun \
  --rerun-web \
  --rerun-open web \
  eval run \
  dimos/benchmark/realtime_sim/cases/go2-apartment-go-to-bed/case.json \
  --output=/tmp/dimos-eval-go-to-bed
```

Open <http://localhost:7779> if the browser does not open automatically. Use
`uv run dimos --viewer none eval run ...` for explicit headless execution.
Viewer options alter presentation only; they do not alter the case fingerprint,
agent task, deadline, private validator, or outcome rules.

The case already binds PiMSim, its scene, Go2, the blueprint, preparation, and
the deadline. There is intentionally no `--runtime` or source override.

- `completed/passed`: the private goal was observed by the deadline.
- `completed/failed`: the runtime stayed healthy but the final deadline sample
  did not satisfy the goal.
- `failed/not_evaluated`: provider, startup, preparation, agent, validator, or
  required live state failed, so no trustworthy task result exists.

Artifacts under the printed attempt path contain startup and preparation
receipts, Pi and CodePolicy evidence, private goal observations, cleanup
diagnostics, a manifest, and one normalized outcome. Public CLI output omits the
private query, target bounds, distance, and threshold.

If preflight reports that `pimsim` is missing, install both the PiMSim package
and its `integrations/dimos` package into the same environment as DimOS. A scene
startup error often means the apartment assets have not been prepared or the
host lacks MuJoCo rendering support. A readiness or preparation error is an
infrastructure failure; inspect `events.jsonl`, `runtime-startup.v1.json`, and
`source-preparation.v1.json` in the attempt directory.

## What remains the same

Frozen QA and long-horizon evaluation share the same outer structure:

```text
                         FROZEN QA              LIVE LONG-HORIZON
                         ---------              -----------------
source                   recording snapshot    simulator/real robot
memory                   read-only, fixed       read-only, updating
DimOS control (`app`)     absent                 available
agent workspace          CodePolicy             CodePolicy
agent loop               Pi                     Pi
typical interaction      query and answer       observe and act repeatedly
validator                final answer check     ongoing/final world check
evidence                 full attempt record    full attempt record
```

Both modes create fresh agent and CodePolicy sessions for each attempt. Both
keep the validator private. Both retain enough evidence to reconstruct what the
agent saw, queried, answered, and did.

## Why CodePolicy is standalone

CodePolicy does not need to be a robot module. Running it as a standalone
service gives us a cleaner ownership boundary:

```text
evaluation runner owns                  DimOS owns
----------------------                  ---------
Pi session                              sensors
CodePolicy workspace                    perception
agent tool connection                   mapping
attempt evidence                        navigation/control
validation lifecycle                    robot runtime
```

For frozen QA, this removes the need to start a DimOS blueprint at all. For a
live evaluation, CodePolicy connects to the already-running blueprint through
Porcelain. The evaluated agent remains outside the robot runtime while still
using the robot's supported interface.

## Evidence and privacy

Each attempt records the public case, selected source, agent condition,
CodePolicy calls, Pi session, final response, parsed prediction, private score,
timing, and terminal outcome.

Operational completion and task correctness remain separate:

```text
agent and infrastructure completed + correct result   = completed / passed
agent and infrastructure completed + wrong result     = completed / failed
infrastructure failed before evaluation finished      = failed / not evaluated
```

The case fingerprint protects the identity of the authored evaluation. Private
validator material stays out of the agent prompt, Memory2 namespace, CodePolicy
results, public logs, and public outcome.

## Further reading

The OpenSpec change contains the detailed proposal and contracts:

- [Archived proposal](/openspec/changes/archive/2026-08-05-standalone-code-policy-frozen-qa-eval/proposal.md)
- [Archived design](/openspec/changes/archive/2026-08-05-standalone-code-policy-frozen-qa-eval/design.md)
- [General evaluation case model](/openspec/specs/agent-evaluation-case-model/spec.md)
- [Frozen QA evaluation spec](/openspec/specs/frozen-qa-agent-evaluation/spec.md)
- [Standalone CodePolicy service spec](/openspec/specs/standalone-code-policy-service/spec.md)
- [Short-horizon QA runbook](/docs/benchmark/short-horizon-qa.md)

Start with the proposal for the motivation and scope. Read the design for the
architecture and the planned transition from frozen QA to live evaluations.
