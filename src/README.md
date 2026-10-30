# Swerve v2 (VEX V5, PROS 4 C++)

A clean rewrite built from the three design documents (Input Smoothing and
Steering Methodology, Pod Rotation Coupling Compensation, Navigation Guide), with
the corrections found while comparing them to the v1 code. Nothing is copied from
v1 except the port numbers, geometry, and motor reversal flags.

## Install

1. Create a **new** PROS project (`pros conduct new-project swerve_v2` or the VS Code extension).
2. Copy `include/swerve/` into the project's `include/` folder.
3. Copy everything in `src/` into the project's `src/`, replacing the template `main.cpp`.
4. `pros make`, upload.

`tests/` is optional and never part of the robot build. On a computer with g++:
`bash tests/run_tests.sh` runs 115 checks and a closed-loop module simulation.

## What the code does (one 10 ms cycle)

```
controller snapshot
 -> radial deadband, response curve, polar slew limits      input_conditioner.hpp
 -> ChassisCommand {vx, vy, omega} in m/s, rad/s
 -> four-module inverse kinematics + ONE global normalize   kinematics.hpp
 -> per module:                                             module_controller.hpp
      low-speed angle hold + flip optimizer w/ hysteresis   module_optimizer.hpp
      pod error -> profiled pod rate (rate + accel limits)  steering_profile.hpp
      cosine alignment scale (intended wheel speed ONLY)    drive_compensation.hpp
      drive RPM = base + coupling cancellation (unscaled)   drive_compensation.hpp
 -> compute all four, THEN commit all eight motors          swerve_drive.cpp
```

Files: `config.hpp` holds every number. `swerve_module.cpp` / `swerve_drive.cpp` are
the only code that touches motors. `display.cpp` runs the controller power display
and brain diagnostics in a low-priority task, so printing can never stall the loop.

## Where this differs from the documents (and why)

| Topic | Documents | v2 |
|---|---|---|
| Coupling coefficient | 2.5 drive rev per pod rev, 1.25 induced wheel rev | **1.0 and 0.5.** The wheel rolls according to its speed relative to the pod, so the only thing that matters is the center gear's speed relative to the pod. The drive motor spinning at the pod's rate cancels it, for any tooth counts. This matches the 0.5 measured on v1. |
| Flip applied to coupling | Doc 2 multiplies by the flip again | Optimizer's signed speed is used once; coupling follows the pod only (guide C1/C2). |
| Steering actuator | Custom voltage control | **Cascaded:** a profiled pod rate sent as motor velocity. The firmware velocity loop is already tuned, so only one gain (`kPodRateKpPerSec`) needs tuning, and the profiled rate doubles as Doc 2's low-lag "requested pod rate" (guide C10 allows a velocity interface). |
| Drive limit | Nominal cartridge RPM | A reserve for full-rate pod cancellation is held back, so compensation always has authority (guide C6/C7/C9). Base limit is 156 of 200 RPM, about 13.3 in/s. |
| Saturation | Priority not locked | Compensation first; lost base/coupling reported separately. |

## Bring-up checklist (do these in order; wheels off the floor first)

1. **Ratio check (autonomous).** Pods centered at start, run autonomous (drives for
   "1 m" at a fixed speed). Measure the real distance. Not about 1.0 m means the
   wheel diameter or drive ratio in `config.hpp` is wrong. v1 had stage 1 teeth
   swapped (ratio 0.889 instead of 2.0); a `static_assert` now guards this.
2. **Homing.** Center all pods by hand, press X. The drive is inhibited until then.
3. **Pod sign.** Push the left stick forward. All pods should stay at 0. Strafe
   right: all pods go to 90 degrees. If a pod turns the wrong way, flip that module's
   `steerReversed` in `config.hpp`.
4. **Drive sign.** Wheels forward = robot forward on every module (`driveReversed`).
5. **Spin pinwheel.** Push the right stick right. On the brain screen the pod targets
   should read FL 45, FR 135, RR 225, RL 315, and the robot should spin clockwise.
   If the targets are right but the robot spins the other way, your left/right
   module positions in `config.hpp` are mirrored; fix the positions.
6. **Coupling sign, per module.** Wheel off the floor, drive stopped. Hold a small
   stick input that swings the pod and watch the wheel. It should stay still. If it
   spins MORE, set that module's `couplingSign` to -1. Do each module.
7. **Coupling size.** With the sign right, the residual wheel motion should be small.
   If it undershoots or overshoots, adjust `kRequestedRateWeight` (0.5) first; only
   touch `kCancelMotorRevPerPodRev` if a one-revolution hand test says so (expect
   0.5 wheel rev per pod rev with the drive motor held still).
8. **Floor test**, then tune in this order: `kPodRateKpPerSec`, pod accel/rate limits,
   then the slew limits last.

## Tuning knobs worth knowing

- `kPodRateKpPerSec` (10): higher = snappier pods, too high oscillates.
- `kPodMaxRateDegPerSec` (240): faster pods need more drive reserve (lower top speed).
- `kRequestedRateWeight` (0.5): 1.0 trusts the profile, 0.0 trusts the encoder.
- `kTranslationAccelPerSec` / `kTranslationDecelPerSec`, `kDirectionSlewDegPerSec`.
- `kRotationAuthority` (1.0): lower it if spinning swamps translation when combined.

## Assumptions to confirm

- Pods may rotate continuously (no cable limit). v1 kept the pod in the front
  half-circle; if your pods can wind up cables, tell me and I will add a travel limit.
- The drive motor is fixed to the chassis (its shaft runs up the pod's rotation axis).
- Top speed is about 13 in/s with green cartridges and this gearing. A blue
  drive cartridge (`kDriveCartridge`) is a one-line change.

## Not built yet (deferred in Doc 1 until base behavior passes)

X-lock mode, field-relative control, odometry, indexed or absolute homing sensor,
wheel-speed residual trim, and logging to the SD card.
