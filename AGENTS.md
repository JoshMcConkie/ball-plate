# AGENTS.md — Engineering Agent Operating Policy

Read this file before modifying the project.

## 1. Prime Directive

Use AI to compress implementation work **without silently changing the engineering model, design intent, or experimental hypothesis**.

The agent may autonomously repair code that fails to express the stated design.
The agent must **surface, not silently repair**, evidence that the stated design itself may be wrong.

Optimize for:
1. correctness,
2. preservation of intent,
3. small and interpretable changes,
4. useful verification,
5. fast iteration without hiding informative failures.

A working demo is not sufficient if the change obscures why the system works.

---

## 2. Authority Boundary

### A. Implementation/mechanical issue — MAY FIX

An issue is in this class when the intended behavior is already clear and the code fails to express it.

Typical examples:
- syntax or parse errors;
- missing or incorrect imports;
- type or shape mismatches with an unambiguous intended type/shape;
- misspelled names or wrong variable references;
- incorrect use of a documented API;
- obvious argument-order mistakes;
- unambiguous indexing/off-by-one mistakes;
- serialization/parsing defects;
- build, packaging, or configuration syntax defects;
- mechanical wiring between already-defined interfaces;
- tests or logging needed to verify the existing intended behavior.

For these issues:
- make the smallest reasonable fix;
- preserve architecture and behavior outside the defect;
- verify the fix;
- explain the cause briefly.

### B. Model/design issue — DO NOT AUTONOMOUSLY CHANGE

An issue is in this class when correcting it would change what the system is supposed to mean or how it is intended to behave.

Protected design decisions include, unless the user explicitly authorizes changes:
- equations, dynamics, objective functions, or physical models;
- state definitions or state dimension;
- measurement models;
- coordinate frames, transforms, sign conventions, or units;
- process/measurement noise assumptions or covariance structure;
- estimator/filter family or structure;
- controller family, gains, cost functions, or tuning policy;
- calibration method or calibration parameters;
- sampling/control/estimation rates and timing architecture;
- sensor interpretation or fusion strategy;
- actuator command semantics, limits, saturation, or safety behavior;
- learned-model architecture, loss, reward, training objective, or data semantics;
- public interfaces between major subsystems;
- concurrency/process architecture;
- dependencies or major abstractions;
- product requirements or externally observable behavior.

If one of these appears responsible:
1. do not tune or redesign it merely to make the failure disappear;
2. report the evidence;
3. state the suspected design/model issue as a hypothesis;
4. identify what observation or experiment would distinguish it from an implementation bug;
5. wait for explicit authorization before changing it.

### C. Ambiguous issue — STOP AND REPORT

If a fix depends on guessing the intended behavior, classify it as ambiguous.

Do not choose an interpretation just because it makes tests pass.

Report:
- what is ambiguous;
- the plausible interpretations;
- what evidence exists for each;
- the smallest decision needed from the user.

---

## 3. Debugging Protocol

When asked to debug or fix a failure:

1. **Identify the observed failure.**
   - Distinguish the actual symptom from inferred causes.

2. **Recover the stated intent.**
   - Read the relevant project documentation, tests, interfaces, comments, and nearby code.
   - Prefer explicit project contracts over assumptions.

3. **State the invariant.**
   - What behavior is supposed to remain true?

4. **Classify each suspected issue.**
   - `A — implementation`
   - `B — model/design`
   - `C — ambiguous`

5. **Patch only Class A issues by default.**
   - Keep the diff focused.
   - Do not bundle unrelated cleanup.

6. **Verify proportionally to the change.**
   - Run the narrowest relevant test first.
   - Expand verification when the risk or scope warrants it.

7. **Report separately.**
   - implementation fixes made;
   - verification performed;
   - remaining model/design hypotheses;
   - unresolved ambiguity.

If a Class A fix does not resolve the observed behavior, do **not** start changing Class B parameters to chase success.

---

## 4. Preserve the Experiment

For experimental, scientific, robotics, control, ML, simulation, or hardware projects, failures may contain useful information.

Do not erase an informative discrepancy between theory, simulation, and reality by silently:
- tuning gains;
- inflating/deflating noise values;
- changing time steps or rates;
- altering calibration;
- adding smoothing;
- changing model structure;
- modifying loss/reward functions;
- weakening tests or acceptance thresholds.

Instead, expose the discrepancy.

Prefer the loop:

**hypothesis → implementation → observation → diagnosis → revised hypothesis**

Avoid the loop:

**prompt → error → regenerate → repeat until green**

---

## 5. Minimal-Change Discipline

For debugging and maintenance:
- make one conceptual change at a time when practical;
- avoid drive-by refactors;
- avoid speculative abstractions;
- avoid renaming/reformatting unrelated code;
- avoid new dependencies unless required and justified;
- do not combine a refactor with a behavioral fix unless separation is impractical;
- preserve known-good interfaces unless the task explicitly changes them.

A smaller diff is preferred when it provides the same correctness and maintainability.

If a broader redesign is clearly beneficial, propose it separately from the immediate repair.

---

## 6. Verification Rules

Verification should demonstrate that the intended behavior was restored, not merely that the current test suite is green.

Do:
- use existing tests and project commands where available;
- add focused regression tests for implementation defects when appropriate;
- inspect logs, outputs, dimensions, units, timestamps, or traces relevant to the failure;
- compare before/after behavior when useful;
- note what was **not** tested.

Do not:
- weaken assertions to make a test pass;
- delete failing tests without explicit justification;
- mock away the behavior under investigation;
- change expected values merely to match current output;
- hide warnings/exceptions that reveal the original problem;
- claim physical or end-to-end verification when only static/unit checks ran.

For physical systems, simulation success is not evidence of hardware success.

---

## 7. Physical-System and Safety Guardrails

Unless explicitly authorized, never change:
- actuator limits;
- emergency-stop/fail-safe behavior;
- current/voltage/temperature limits;
- collision limits;
- watchdog behavior;
- command ranges;
- unit conventions;
- hardware enable/disable behavior;
- motion direction/sign conventions.

Do not execute commands that can move hardware, energize actuators, erase calibration, flash firmware, or alter safety-critical configuration unless the task clearly authorizes that action.

Prefer inspection, simulation, dry runs, or read-only diagnostics before physical actuation.

---

## 8. Learning and Explainability

When making a nontrivial fix, preserve enough explanation for a technically competent maintainer to understand the causal chain.

The agent should be able to answer:
- What was wrong?
- Why did the change fix it?
- Which assumption made the fix valid?
- What behavior should change?
- What important behavior should remain unchanged?

Do not replace a clear engineering explanation with “best practice,” “more robust,” or “optimized” without specifying the mechanism.

When introducing mathematics or derived parameters:
- state the assumed model;
- state relevant units and dimensions;
- distinguish derived values from tuned values;
- identify assumptions supplied by the user versus inferred by the agent.

---

## 9. Tuning Is Not Debugging

Do not use parameter tuning as a substitute for identifying an implementation defect.

Examples of protected tuning targets:
- PID/LQR/MPC parameters;
- Kalman `Q`, `R`, or initial covariance;
- thresholds and tolerances;
- filtering/smoothing constants;
- loop frequencies;
- optimizer hyperparameters;
- learning rates;
- reward weights;
- calibration constants.

If tuning is explicitly requested:
- identify the performance metric;
- preserve safety constraints;
- change a limited number of variables at once;
- retain the previous values;
- report the effect rather than merely the final setting.

---

## 10. Generated Code Is Not Automatically Trusted

Treat generated code as a proposal.

Before relying on it:
- check that it respects project interfaces and invariants;
- inspect assumptions at subsystem boundaries;
- verify units, dimensions, rates, ownership/lifetimes, and error handling where relevant;
- prefer existing project patterns over invented frameworks.

Do not rewrite stable code simply because another implementation appears cleaner.

---

## 11. Project Contract — Ball-on-Plate Testbed

This section records the current engineering intent for the Ball-on-Plate project.

**Source-of-truth rule:** this section protects architecture, semantics, assumptions, and interfaces.
For runtime/tunable values, **`data/system_config.json` is the authoritative source of truth**.

Agents must read `data/system_config.json` before reasoning about or changing:
- loop/sample rates;
- serial port, baud, or timeout;
- camera device/configuration;
- reference/setpoint values;
- plate dimensions;
- controller gains/filter coefficients/tilt limits;
- servo geometry, center/min/max angles, GPIO pins, or pulse widths;
- IMU stream rate;
- any other value represented in that configuration file.

Do not duplicate those values into this file, source code, tests, documentation, or prompts unless duplication is explicitly required by the task.

If code, documentation, or another configuration source disagrees with `data/system_config.json` about an active configurable value, **do not silently choose one**. Report the conflict as `C — ambiguous` unless the repository clearly documents a different precedence rule.

### Purpose
- **System purpose:** A modular physical and simulated ball-on-plate testbed for hands-on experimentation with feedback control, state estimation, calibration, sensing, simulation, and later learned/adaptive robotics methods.
- **Primary success criterion:** The system should reliably observe the ball and plate, estimate the relevant state, and command bounded plate motion to regulate or move the ball while keeping the causal chain inspectable and measurable.
- **Secondary purpose:** Make control/estimation components swappable so classical and learned/adaptive methods can be compared without silently changing the surrounding system.

### Architecture

#### Physical system
```text
Camera
  -> vision / camera calibration
  -> ball position measurement
  -> ball-state estimator
                             \
                              -> controller
                             /      -> desired plate tilt / command
IMU -> plate attitude fusion       -> actuator mapping / calibration
                                      -> host-to-ESP32 serial
                                      -> ESP32 servo actuation
                                      -> physical plate
                                      -> physical ball
                                      -> sensors again
```

#### Simulation system
```text
same logical state / command interfaces
  -> MuJoCo plant
  -> simulated plate + ball dynamics
  -> simulated observations
  -> estimator / controller
```

Simulation is a development and comparison environment. It is **not** ground truth for the physical plant.

### Major Components
- Host-side application: Python/C++ as currently implemented in the repository.
- Vision: OpenCV ball detection/tracking plus camera-to-plate calibration.
- Ball estimator: 4-state Kalman-filter-oriented pipeline.
- Plate attitude estimation: IMU-based attitude fusion.
- Controller: classical PID/PD baseline unless a task explicitly changes controller family.
- Actuation: ESP32-driven servos and mechanical linkage.
- Communication: host ↔ ESP32 serial.
- Simulation: MuJoCo model of the plate, ball, hinges/linkages, sensors, and actuation.
- Logging/configuration: preserve current project configuration and telemetry interfaces.

### Data / Control Flow
1. Camera produces image frames.
2. Vision extracts a ball position measurement in the calibrated plate/global working frame.
3. IMU produces accelerometer and gyroscope measurements used to estimate plate attitude.
4. The estimator maintains ball state.
5. The controller uses target state + estimated ball/plate state to compute a bounded plate command.
6. Actuator mapping converts the plate command into servo commands.
7. ESP32 applies servo commands.
8. The plate/ball evolve physically.
9. Measurements and commands are timestamped/logged for diagnosis and comparison.

Do not bypass this flow merely to make a demo work. A learned component may replace or augment a block only when the task explicitly authorizes that experiment.

---

### Engineering Model

#### Ball State
Current protected core representation:

\[
\mathbf{x}_{ball}
=
[x,\ y,\ \dot{x},\ \dot{y}]^T
\]

- `x`, `y`: ball position on the plate, intended physical units of meters after calibration.
- `xdot`, `ydot`: ball velocity along the plate axes, intended units of meters/second.
- Camera observations directly measure ball position, not velocity.

Do not add acceleration, actuator state, latent variables, or other states merely because they may improve performance. That is a model/design proposal.

#### Ball Measurement Model
- Primary ball measurement: calibrated camera-derived `(x, y)`.
- Measurement covariance `R` is intended to be grounded in observed camera-tracking variation, including stationary-ball sampling.
- Do not invent or retune `R` while debugging an unrelated problem.

#### Ball Process Model
The current project direction is physics-based state propagation for the 4-state ball model, using elapsed time and plate attitude/tilt where implemented.

Important:
- The exact physics approximation, rolling coefficient/constant, friction treatment, and control-force treatment are experimental model assumptions.
- Do not replace the current process model with a different dynamics model as an implementation fix.
- `Q` represents process/model uncertainty and is expected to depend on the assumed disturbance model and `dt`; it is not a generic "make the filter work" knob.
- If current behavior suggests the process model is inadequate, report that as `B — model/design`.

#### Plate / IMU Representation
The IMU provides:
- accelerometer axes `(a_x, a_y, a_z)`, conceptually in m/s² after unit conversion;
- gyroscope axes `(g_x, g_y, g_z)`, with the code responsible for explicit and consistent angular-rate units.

The project distinguishes:
- **local plate roll/pitch and local plate axes**, used in attitude estimation;
- **global/external plate-angle representation**, used to describe the physical plate orientation relevant to ball dynamics/control.

Frame conventions are still an area requiring care. Do not "clean up" roll/pitch, axes, signs, or local/global transforms unless the task explicitly addresses frame semantics.

Historical design notes may use different IMU state parameterizations. Read the current implementation before assuming a particular internal attitude-filter state vector.

#### Outputs / Commands
The controller conceptually outputs a desired bounded plate orientation/tilt, which is converted through the existing actuator mapping/calibration into servo commands.

Read active controller, reference, and actuator values from `data/system_config.json`, including:
- `reference.*`
- `controller.*`
- `servos.*`

Protected semantics:
- plate-command axes;
- command sign;
- unit convention;
- neutral position;
- actuator mapping;
- servo limits;
- saturation behavior.

Do not reinterpret a command field just because another convention appears more natural.

#### Coordinate Frames / Conventions
Known intent:
- ball coordinates are expressed relative to the plate/workspace after camera calibration;
- IMU raw measurements are in the IMU/plate-local sensor frame;
- local roll/pitch must not be casually conflated with an external/global plate-angle representation;
- camera, plate, IMU, and actuator frames require explicit transforms.

Current exact axis/sign conventions must be read from the repository/configuration.

If two modules disagree on signs, frame direction, or axis naming:
- do not choose whichever version makes the test pass;
- report the conflict as `C — ambiguous` unless documentation clearly establishes the intended convention.

#### Noise / Uncertainty
Protected assumptions:
- camera measurement uncertainty belongs in measurement modeling (`R`);
- unmodeled dynamics/disturbances belong in process modeling (`Q`);
- `Q` and `R` are engineering/model parameters, not generic debugging knobs;
- measured timing variation must not be hidden by silently assuming a fixed `dt` if the implementation uses timestamps.

---

### Timing

All active runtime rates and timing-related serial settings must be read from:

`data/system_config.json`

Relevant keys currently include:
- `runtime.rates_hz.camera_capture`
- `runtime.rates_hz.imu_read`
- `runtime.rates_hz.state_estimation`
- `runtime.rates_hz.control`
- `runtime.rates_hz.servo_command`
- `runtime.rates_hz.debug_output`
- `serial.timeout_s`
- `imu.firmware_stream_rate_hz`

Rules:
- Do not hardcode numeric rates in this file or infer that a rate is "optimal."
- Preserve the multi-rate architecture unless the task explicitly changes it.
- Do not change loop rates while debugging an unrelated problem.
- Preserve actual timestamps and measured `dt` where the implementation supports them.
- If runtime behavior differs from `data/system_config.json`, diagnose the discrepancy rather than silently changing the config or code.

---

### Physical Geometry / Simulation Parameters

Physical-system geometry and actuator configuration that are represented in the project configuration must be read from:

`data/system_config.json`

Relevant keys currently include:
- `plate.width_m`
- `plate.height_m`
- `servos.arm_length_m`
- `servos.center_deg`
- `servos.{a,b}.min_deg`
- `servos.{a,b}.max_deg`
- `servos.{a,b}.gpio_pin`
- `servos.{a,b}.min_pulse_us`
- `servos.{a,b}.max_pulse_us`

Do not duplicate those numeric values here.

Simulation-specific geometry or solver parameters that are **not** represented in `data/system_config.json` belong in the MuJoCo model or dedicated simulation configuration/documentation, not in this agent policy.

Simulation parameters are model assumptions, not automatically authoritative measurements of the physical hardware. Do not modify them merely to improve visual behavior without identifying the mismatch being modeled.

Known simulation issue/area of investigation:
- linkage/tendon/equality-constraint compliance can introduce artificial stretch/compression;
- changing constraint stiffness, solver settings, or other simulation-only parameters is a simulation-model change, not a syntax fix.

---

### Safety / Hard Constraints

Before any physical actuation or reasoning about hardware limits, read the active limits from:

`data/system_config.json`

Relevant protected configuration includes:
- `controller.max_tilt_deg`
- `servos.center_deg`
- `servos.{a,b}.min_deg`
- `servos.{a,b}.max_deg`
- `servos.{a,b}.min_pulse_us`
- `servos.{a,b}.max_pulse_us`
- actuator geometry represented under `servos`

Unless explicitly authorized:
- Do not increase configured motion/tilt limits.
- Do not alter servo min/max commands, neutral positions, linkage limits, or saturation behavior.
- Do not alter emergency-stop/watchdog/fail-safe logic.
- Do not reverse actuator signs or axes.
- Do not flash firmware, erase calibration, or energize/move hardware merely to verify a software fix.
- Do not run unattended physical motion tests.
- Do not command motion when telemetry/frame semantics are uncertain.
- Prefer neutral/disabled, dry-run, logged, simulation, or read-only verification before bounded hardware actuation.

Controller gains, filter coefficients, and estimator covariances are protected tuning/model parameters.
Controller values represented in `data/system_config.json` must not be copied here or silently retuned during unrelated debugging.

---

### Interfaces That Must Remain Stable

#### Host ↔ ESP32
- Read serial port, baud rate, and timeout from `data/system_config.json` (`serial.*`).
- Preserve the existing message/field ordering and units defined by the repository.
- Current host-side telemetry parsing has used newline-terminated records with six whitespace-separated fields; do not change that protocol unless the task explicitly changes the serial interface.
- Preserve partial-line buffering behavior; incomplete serial records must not be treated as complete samples.

#### Vision → Estimator
- Preserve the calibrated meaning and units of camera-derived ball position.
- Do not substitute pixel coordinates where physical coordinates are expected.

#### IMU → Attitude Estimator
- Preserve accelerometer/gyro field meaning, axis ordering, timestamps, and units.
- Do not silently convert degrees/s ↔ radians/s or change axis signs.

#### Estimator → Controller
- Preserve state ordering `[x, y, xdot, ydot]` unless an explicit design task changes it.
- Preserve units.

#### Controller → Actuator Mapping
- Preserve command axis semantics, units, saturation, and neutral convention.
- Physical servos use frame-neutral identities `A` and `B`.
- Preserve the current real-system mapping and serial order: the X-acceleration
  command channel is sent first to servo A, and the Y-acceleration command
  channel is sent second to servo B.
- The A/B identities do not establish roll/pitch or simulation-axis semantics.

#### Simulation ↔ Real-System Abstraction
- Prefer compatible logical state/command interfaces so algorithms can be compared across simulation and hardware.
- Do not force identical behavior by hiding physical/simulation mismatch.

---

### Known Intentional Limitations / Open Model Questions

These are **not automatic bugs**:

- The 4-state ball model is intentionally simple.
- Ball/plate dynamics are approximations; friction, rolling effects, actuator dynamics, backlash, linkage compliance, and latency may be incompletely modeled.
- Camera calibration and local/global frame handling are active engineering concerns.
- Exact `Q`/`R` values and uncertainty models are experimental.
- The physical system is multi-rate.
- The estimator/controller baseline is classical by design; learned/VLA/adaptive components are experiments, not default replacements.
- Simulation-to-reality mismatch is expected and should be measured rather than hidden.
- MuJoCo linkage compliance/constraint tuning remains an area of investigation.
- Not every physical imperfection should immediately be "fixed" in software; some are valuable experimental disturbances.

---

### Verification

The repository's actual commands/scripts take precedence. **Do not invent command names.**

#### Build
- Python/C++ host and ESP32 firmware build procedures must be taken from the repository/README/tooling.
- If no standardized command exists, report that rather than creating a new build system during an unrelated fix.

#### Unit Tests
- Use existing unit tests if present.
- Favor focused tests for:
  - state ordering/dimensions;
  - unit conversions;
  - coordinate transforms;
  - serial parsing;
  - timestamp/`dt` handling;
  - estimator matrix dimensions;
  - saturation/command bounds;
  - calibration transforms.

#### Integration Tests
Prefer logged or synthetic-data verification before hardware:
- feed known camera/IMU samples through the pipeline;
- verify state units/order;
- verify neutral/bounded controller output;
- verify serial encoding/decoding;
- verify multi-rate timing behavior without changing configured rates.

#### Simulation
- Use the repository's MuJoCo entry point/model.
- Verify that the simulation loads, remains numerically stable for the tested case, and preserves expected state/command semantics.
- Record any model-parameter or constraint changes explicitly.
- Simulation success does not count as hardware validation.

#### Hardware / End-to-End
**Manual authorization is required before an agent initiates physical actuation.**

Before motion:
1. confirm current safety limits from active configuration;
2. confirm serial connection and telemetry parsing;
3. confirm units, axes, and neutral command;
4. keep plate/robot workspace clear;
5. begin with neutral or minimal bounded commands;
6. log measurements, estimates, and commands.

Do not claim the closed loop is validated unless it was actually tested on the physical system.

---

### Configuration Discipline

`data/system_config.json` is intended to prevent configuration drift.

Agents must:
- read from it rather than copy active values into new code;
- prefer passing configuration values through existing project interfaces;
- avoid introducing a second source of truth;
- preserve the meaning of each config key;
- treat schema changes as design/interface changes, not mechanical fixes;
- report unused, ignored, shadowed, or overridden config values when discovered.

If a value should become configurable but is not yet in `data/system_config.json`, propose that change separately rather than silently adding another hardcoded constant.

---

### Relevant Documentation / Files

Prefer these sources, in this order, when available:

1. `data/system_config.json` — authoritative active runtime/tunable values represented by the schema.
2. Current implementation and tests — actual interface behavior and enforcement of those values.
3. `Estimation Structure.md` — derivations and design notes for ball/IMU estimation; treat explicitly marked cautions or unfinished sections as non-final.
4. Current README / architecture notes.
5. Current MuJoCo model and simulation notes.
6. ESP32 firmware and serial-protocol definitions.
7. Datasheets for the camera, IMU (LSM6DSO where applicable), servos, and other hardware.

Do not treat an old notebook/design note as more authoritative than clearly newer project code/config. If the difference changes engineering meaning rather than merely implementation detail, report it.

---

## 12. Expected Completion Report

For debugging or implementation work, finish with a compact report:

### Changed
- `<small list of implementation changes>`

### Why
- `<causal explanation>`

### Verified
- `<tests/checks actually performed>`

### Model/design concerns not changed
- `<suspected Class B issues, or "none observed">`

### Ambiguities / remaining uncertainty
- `<Class C items, limitations, or "none">`

Do not describe an unverified hypothesis as a confirmed root cause.

---

## 13. Default Decision Rule

When uncertain whether a proposed change crosses from implementation into engineering intent:

**Do not make the change. Surface it as a hypothesis.**

The purpose of this policy is not to make agents passive. It is to make fast iteration interpretable: automate mechanical work aggressively while keeping consequential modeling and design decisions visible to the human engineer.
