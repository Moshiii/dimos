## ADDED Requirements

### Requirement: Fresh declarative Pink backend injection
The generic Quest task factory SHALL accept a `PinkIK` backend class and construct a fresh instance with the task's resolved Pink configuration for every control-task instance.

#### Scenario: Existing declaration uses the default backend
- **WHEN** a Quest task declaration omits the backend class
- **THEN** the factory SHALL construct plain `PinkIK` and preserve existing Quest behavior

#### Scenario: Repeated custom declarations own independent state
- **WHEN** the same Quest declaration using a custom backend class is instantiated twice
- **THEN** each control task SHALL receive a distinct backend instance and mutable Pink task state SHALL NOT be shared

### Requirement: OpenArm translation-dominant frame tracking
The OpenArm Quest backend SHALL retain both mandatory grasp-frame objectives with position cost `1.0` and orientation cost `0.2`.

#### Scenario: Bimanual OpenArm stack is constructed
- **WHEN** the OpenArm Quest task first solves left and right grasp-frame targets
- **THEN** both frame tasks SHALL participate with the configured five-to-one position-to-orientation cost ratio

### Requirement: OpenArm dynamic weighted posture
The OpenArm Quest backend SHALL retain the common per-tick measured/current posture target while applying mirrored per-joint costs equal to `1e-3 * [4, 3, 0.1, 3, 1, 1, 0.1]` for each arm.

#### Scenario: Canonical zero begins at elbow limits
- **WHEN** bimanual streaming starts from the all-zero OpenArm pose
- **THEN** the common inward posture target SHALL move both elbow objectives away from their lower limits without changing the hardware initial pose

### Requirement: OpenArm grasp-frame manipulability
The OpenArm Quest backend SHALL add one native position-only Pink manipulability objective per commanded grasp frame with cost `0.005` and desired rate `0.05`.

#### Scenario: Dual-arm objective is solved
- **WHEN** both OpenArm grasp frames are commanded in one control context
- **THEN** the solver SHALL receive one independently named manipulability task for each frame after the inherited frame and posture objectives

### Requirement: Common Quest and safety semantics remain fixed
Using the OpenArm backend SHALL NOT change Quest engagement, synchronization, per-hand timeout, E-stop, gripper mapping, joint mapping, limit enforcement, or command-delta safety.

#### Scenario: One controller becomes stale
- **WHEN** either controller stream exceeds the Quest timeout while both hands were engaged
- **THEN** the generic Quest task SHALL disengage atomically and SHALL NOT issue another IK command

#### Scenario: Customized solve produces an unsafe command
- **WHEN** the OpenArm backend returns non-finite, out-of-limit, or excessive-delta joint output
- **THEN** the shared pose-target and Pink safety paths SHALL reject or saturate it identically to plain `PinkIK`

### Requirement: Streaming Pink solves honor the task command budget
The common pose-target task SHALL pass its configured per-tick joint-delta limit into streaming Pink solves, and Pink SHALL constrain controlled joint displacement to 99% of that limit while preserving tighter model velocity limits.

#### Scenario: URDF velocity permits a larger command
- **WHEN** the model velocity limit permits more displacement than the pose-target task's per-tick limit
- **THEN** the Pink QP SHALL return a coordinated solution below the task limit instead of relying on post-solve rejection

#### Scenario: OpenArm uses its teleoperation command budget
- **WHEN** the OpenArm Quest task is constructed
- **THEN** its outer command budget SHALL be `10` degrees and its Pink QP budget SHALL be `9.9` degrees per tick

#### Scenario: Quest arm teleoperation limits normal streaming velocity
- **WHEN** a single-arm or dual-arm Quest task omits an explicit streaming velocity
- **THEN** every controlled joint SHALL default to at most `1.0 rad/s` in addition to its URDF velocity and command-displacement limits

#### Scenario: A robot tunes its Quest streaming velocity
- **WHEN** a robot Quest declaration supplies `max_joint_velocity_rad_s`
- **THEN** the task SHALL use that positive finite value instead of the `1.0 rad/s` default

#### Scenario: Model velocity is already tighter
- **WHEN** a model velocity limit permits less displacement than the task command budget
- **THEN** the Pink QP SHALL retain the tighter model velocity constraint

#### Scenario: Accepted feedback is normalized near a position limit
- **WHEN** measured feedback lies within tolerance but Pink normalizes it into the command-safe position interval
- **THEN** the constrained solution SHALL remain within the command budget measured from the original feedback

#### Scenario: Planning omits a streaming command budget
- **WHEN** a Pink caller does not provide a per-tick command bound
- **THEN** Pink SHALL retain its existing default configuration and velocity-limit behavior
