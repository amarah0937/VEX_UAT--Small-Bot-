# Swerve v2 Glossary

Every name in the code: what it is, where it lives, and what happens if you change
it. Values shown are the defaults in the current `config.hpp`. "Derived" numbers are
computed from other constants and are not meant to be edited directly.

**Contents**
1. [Reading the names](#1-reading-the-names)
2. [config.hpp constants](#2-confighpp-constants)
3. [Per-module table (`ModuleConfig`)](#3-per-module-table-moduleconfig)
4. [Data types (`types.hpp`)](#4-data-types-typeshpp)
5. [Parameter structs](#5-parameter-structs)
6. [Functions and classes by file](#6-functions-and-classes-by-file)
7. [Internal state variables](#7-internal-state-variables)
8. [Reading the screens](#8-reading-the-screens)
9. [Symptom to knob cheat sheet](#9-symptom-to-knob-cheat-sheet)

---

## 1. Reading the names

**Unit suffixes.** Every number carries its unit in its name.

| Suffix | Unit | Suffix | Unit |
|---|---|---|---|
| `M` | meters | `Rpm` | revolutions per minute |
| `Mps` | meters per second | `Sec` | seconds |
| `Rad` | radians | `Ms` | milliseconds |
| `RadPerSec` | radians per second | `Deg` | degrees |
| `PerSec` | per second (rates of normalized values) | `Rev` | revolutions |

**Angles (compass style).** 0 = straight forward, positive = clockwise (toward the
right). So 90 = right, 180 = backward, 270 = left. Defined in `angle_math.hpp`.

**Directions.** +x is right, +y is forward, spin (omega) positive is clockwise.

**"Pod" versus "motor" units.** The pod is the whole rotating module. The steering
motor turns `kSteerMotorRevPerPodRev` (2.0) times for each pod turn. Everything past
the hardware boundary (`SwerveModule::sample()`) is in POD units, and the 2:1 is
applied in exactly one place going in and one going out.

**"Canonical" direction.** Positive drive RPM means the wheel moves the robot
physically forward, and positive steer RPM turns the pod clockwise, on every module.
The motor reversal flags make that true; nothing else flips signs.

**Normalized.** A number scaled to -1..+1 (joystick values, slew limits).

---

## 2. config.hpp constants

All are in `namespace swerve::config` in `include/swerve/config.hpp`. This is the only
place to put robot-specific numbers.

### 2.1 Geometry

| Name (default) | What it is and where it is used | Raise it | Lower it |
|---|---|---|---|
| `kInchToM` (0.0254) | Unit conversion used only in this file. Do not change. | n/a | n/a |
| `kWheelbaseM` (10.5 in) | Front-to-back distance between pod axes. Feeds `kHalfLengthM`, the module positions, `kModuleRadiusM`. | Pods sit farther from center, so a given spin needs faster wheels: top spin rate drops (`maxOmega = maxWheel / radius`). Pod angles during pure spin stop being exactly 45 degrees. | Opposite. |
| `kTrackwidthM` (10.5 in) | Left-to-right distance between pod axes. Same uses. | As above. | As above. |
| `kHalfLengthM`, `kHalfWidthM` | Derived: half of the two above. Used for the corner positions. | Derived | Derived |
| `kPosFrontLeft`, `kPosRearLeft`, `kPosFrontRight`, `kPosRearRight` | `Vec2` position of each corner, meters from robot center (x right, y forward). Assigned to modules in the module table. | Wrong values make the pods point where the spin would be correct for a different robot: scrubbing and lopsided spin. | Same |

If the geometry is wrong or a module is assigned to the wrong corner, the symptom is a
bad pinwheel during pure spin (see the decoder in the README, step 5).

### 2.2 Mechanics

| Name (default) | What it is and where it is used | Raise it | Lower it |
|---|---|---|---|
| `kDriveCartridge` (`Green200`) | Which cartridge is in the DRIVE motors: `Red100`, `Green200`, `Blue600`. `SwerveModule` turns it into the PROS gearset; `nominalRpm()` turns it into `kDriveLimitRpm`. | Must match the hardware. Setting a faster color than installed makes real speed roughly proportionally slower than commanded. | n/a |
| `kSteerCartridge` (`Green200`) | Same for the STEERING motors. Used for the steer RPM clamp and the `static_assert` that the pod max rate is achievable. | Must match the hardware. | n/a |
| `kWheelDiameterM` (3.25 in) | Effective rolling diameter. With `kWheelCircumferenceM` it converts m/s to wheel RPM. | A bigger number means FEWER RPM per m/s, so the robot really drives slower and shorter than commanded; the 1 m test comes up short. | Opposite: drives farther than commanded. Measure a rollout for best accuracy. |
| `kWheelCircumferenceM` | Derived: pi times the diameter. | Derived | Derived |
| `kDriveStage1DriverTeeth`, `...DrivenTeeth`, `kDriveStage2DriverTeeth`, `...DrivenTeeth` | Tooth counts of the two drive stages, driver then driven. Only used to compute the ratio below. | See next row. | See next row. |
| `kDriveMotorRevPerWheelRev` (0.889) | Derived from the teeth: motor revolutions per wheel revolution with the pod locked. Used in `kMotorRpmPerWheelMps` and `kCancelMotorRevPerPodRev`. **Check it with the 1 m autonomous test:** new = old x (1.0 m / distance driven). | More motor RPM per m/s: lower top speed, and the cancellation gets bigger. A value that is too high makes the robot go shorter than commanded. | Opposite: too low drives farther than commanded and under-cancels pod drag. |
| `kSteerMotorRevPerPodRev` (2.0) | Steering reduction: 40T on the motor driving an 80T on the pod. Converts steering encoder degrees and RPM to pod angle and rate, and pod rate back to motor RPM. | If wrong, pods turn the wrong amount (an angle error that grows with every move) and the pod rate used for cancellation is off. Only change if the gearing changes. | Same |
| `kInducedWheelRevPerPodRev` (0.5) | **Measured.** Hold the drive motor still, turn a pod exactly one revolution by hand, count wheel revolutions. Read by `kCancelMotorRevPerPodRev`. | Bigger cancellation: if too high the wheel spins the OTHER way while pods turn. | Smaller cancellation: if too low the robot creeps while pods turn. |
| `kCancelMotorRevPerPodRev` (0.444) | Derived: `kInduced... x kDriveMotorRevPerWheelRev`. The drive-motor revolutions added per pod revolution to cancel wheel roll. Used in `DriveCompParams`, the reserve below, and `calculateDriveSetpoint()`. | See the two rows above. | See the two rows above. |
| `kMotorRpmPerWheelMps` (205.7) | Derived: `60 x ratio / circumference`. Drive-motor RPM needed per 1 m/s of wheel speed. Used for every base wheel command and for the top speed. | Derived | Derived |

### 2.3 Timing

| Name (default) | What it is and where it is used | Raise it | Lower it |
|---|---|---|---|
| `kControlPeriodMs` (10) | Control loop period, passed to `pros::Task::delay_until` in `main.cpp`. | Slower loop: slews, profile and compensation update less often, more lag and quantization. The slew and profile math uses the real dt, so it still behaves, but it is coarser. | Faster loop: finer control, more CPU. The motors only update every 10 ms internally, so going lower gains little. |
| `kDtMinSec` (0.005), `kDtMaxSec` (0.030) | The measured loop time is clamped into this range (`measureDt()` in `main.cpp`) so one slow or fast cycle cannot make a slew or profile jump. | A higher max lets a long stall move things farther in one cycle. | A tighter range ignores more real timing variation. |

### 2.4 Steering motion

| Name (default) | What it is and where it is used | Raise it | Lower it |
|---|---|---|---|
| `kPodMaxRateDegPerSec` (240) | Fastest the pod is allowed to turn. Used by the steering profile, the steer RPM clamp, and the drive reserve. Hard ceiling: the steering cartridge gives about 600 deg/s (checked by a `static_assert`). | Pods realign faster, but the drive reserve (below) grows, so top speed drops. | Slower realignment, more top speed. Too low and the robot drives with misaligned wheels for longer. |
| `kPodMaxAccelDegPerSec2` (1500) | Max change of pod turn rate per second. Used by the steering profile for both speeding up and braking. | Snappier pod starts and stops; too high gives jerk, overshoot and current spikes. | Smoother but mushier pods that lag the stick. |
| `kPodRateKpPerSec` (10) | Proportional gain: pod rate (rad/s) asked for per radian of error. | Tighter tracking; too high causes the pod to hunt back and forth near the target (visible as steering jitter). | Pods approach slowly and settle late. |
| `kPodSettleToleranceDeg` (0.5) | Below this error the pod is told to stop. | Pods stop earlier and rest up to this far off target; less chatter. | More precise but more hunting when the encoder is noisy. |
| `kPodMaxRateRadPerSec` | Derived: `kPodMaxRateDegPerSec` in radians per second. | Derived | Derived |

### 2.5 Drive limits

| Name (default) | What it is and where it is used | Raise it | Lower it |
|---|---|---|---|
| `kDriveUsableFraction` (1.0) | Fraction of the cartridge's nominal RPM treated as available. | Above 1.0 asks for speed the motor cannot give: commands saturate. | Leaves headroom for battery sag and load, at the cost of top speed. Try 0.9 if the robot feels like it runs out of speed. |
| `kDriveLimitRpm` (200) | Derived: nominal RPM x usable fraction. The hard clamp on the final drive RPM in `calculateDriveSetpoint()`. | Derived | Derived |
| `kReserveMargin` (1.1) | Safety multiplier on the reserve. | Bigger reserve: lower top speed, compensation almost never saturates. | Smaller reserve: more speed, but compensation can be squeezed at full speed with fast pods. |
| `kCompensationReserveRpm` (19.6) | Derived: drive RPM held back so the pod can turn at full rate while the wheel is also driven. | Derived | Derived |
| `kBaseDriveLimitRpm` (180.4) | Derived: `kDriveLimitRpm - kCompensationReserveRpm`. The most RPM the plain wheel request may use. | Derived | Derived |
| `kMaxWheelSpeedMps` (0.877 m/s, 34.5 in/s) | Derived: top ground speed. Used as the stick's full-scale speed (`maxTranslationMps`), the kinematics normalization limit, and the base for the hold thresholds. | Derived. Change it by changing the ratio, diameter, cartridge or reserve. | Derived |
| `kModuleRadiusM` (0.1886) | Derived: distance from robot center to a pod. Used only to set the max spin rate. | Derived | Derived |
| `kRotationAuthority` (1.0) | Fraction of the wheel-speed budget that full right stick may use for pure spin. Sets max spin rate (4.65 rad/s, 267 deg/s at 1.0). | Faster spin; in combined driving, spin eats more of the budget and translation is scaled down more. | Slower spin; combined driving keeps more translation. |

### 2.6 Driver input

| Name (default) | What it is and where it is used | Raise it | Lower it |
|---|---|---|---|
| `kTranslationDeadband` (0.08) | Radial deadband on the left stick, as a fraction of full travel. In `InputConditioner::radialDeadband()`. | Stick noise and drift are ignored more, but there is a bigger dead zone and slightly less fine control near center. | Finer near center, but a worn stick can make the robot creep. |
| `kRotationDeadband` (0.10) | Same for the right stick (rotation). | Same | Same |
| `kCurveLinearWeight` (0.35), `kCurveCubicWeight` (0.65) | The response curve is `linear*x + cubic*x^3` (divided by their sum). More cubic = gentler near center, steeper near full stick. | More cubic: finer low-speed control, abrupt at the top. | More linear: a straight, more sensitive response. Equal weights of 1 and 0 give a perfectly linear stick. |
| `kTranslationAccelPerSec` (3.0) | How fast translation speed may build, in stick-fractions per second. 3.0 = 0 to full in 0.33 s. | Snappier, but wheel scrub and tipping risk. | Smoother, sluggish. |
| `kTranslationDecelPerSec` (5.5) | How fast translation may drop. 5.5 = full to zero in 0.18 s. Kept higher than accel so stopping stays prompt. | Brakes harder. | Coasts longer after you release the stick. |
| `kDirectionSlewDegPerSec` (540) | How fast the translation DIRECTION may rotate. Big reversals (over 90 degrees) instead slow to near zero, switch direction, then speed up. | The robot follows stick sweeps more exactly. | Sweeping the stick lags; the path takes wider arcs. |
| `kRotationAccelPerSec` (4.0) / `kRotationDecelPerSec` (6.0) | Same for the spin command. A sign flip passes through zero first. | Snappier turn start/stop. | Softer. |
| `kRotationSign` (+1) | +1 means stick right spins clockwise. | -1 reverses spin direction. Use ONLY if the pods already form the right pinwheel (README step 5) and the robot still turns the wrong way. | Same |

### 2.7 Module optimization (flip and hold)

| Name (default) | What it is and where it is used | Raise it | Lower it |
|---|---|---|---|
| `kHoldEnterFraction` (0.04) | When a module's requested speed falls below this fraction of `kMaxWheelSpeedMps` (0.035 m/s), it holds its pod angle and stops the wheel. | Holds sooner: fewer pod twitches near zero speed, but the wheel cuts out at low speed. | Holds later: finer slow driving, more twitch from noisy near-zero angles. |
| `kHoldExitFraction` (0.07) | Speed (0.061 m/s) needed to start moving again. Keep it above the enter value; the gap is hysteresis that stops flicker. | A bigger gap needs more stick to restart. | Too close to the enter value and the hold can flicker. |
| `kFlipEnterDeg` (100) | If the pod would have to turn MORE than this to reach the requested direction, the module flips: pod to the opposite angle, wheel reversed. | Pods turn farther before flipping. | Flips sooner; below 90 it fights itself. |
| `kFlipExitDeg` (80) | Flipped modules return to direct when the direct angle is closer than this. Keep it under the enter value. | Flip state flips back sooner. | Stays flipped longer. A gap of less than about 10 degrees risks chatter near 90. |

### 2.8 Coupling cancellation blending

| Name (default) | What it is and where it is used | Raise it | Lower it |
|---|---|---|---|
| `kRequestedRateWeight` (0.5) | Mix of the PROFILE pod rate (instant, but assumes the pod does what it was told) and the MEASURED rate (real, but laggy and noisy) for the cancellation. 1 = profile only. | Less lag, but if the pod is blocked or stalled the cancellation keeps going. | More faithful to reality, more lag and noise. |
| `kActualRateFilterTauSec` (0.03) | Low-pass time constant on the measured pod rate (`ModuleController::step`). | Smoother, but lags more, so the cancellation arrives late. | Less lag, more noise passed to the wheels. |
| `kPodRateDeadbandDegPerSec` (2.0) | Mixed pod rates smaller than this are ignored (no cancellation). | Less encoder-noise jitter on the wheels, but small pod moves leave residual wheel roll. | Cancels tiny moves too, but passes noise. |

---

## 3. Per-module table (`ModuleConfig`)

The struct is in `config.hpp` (outside the `config` namespace). The four instances
`kFL`, `kRL`, `kFR`, `kRR` fill it in and `kModules` holds them in the order FL, RL, FR,
RR (this is also the storage and print order everywhere).

| Field | Meaning | If you change it |
|---|---|---|
| `name` | Label printed on screens. Names are PHYSICAL corners. | Cosmetic. |
| `drivePort` | Smart port of the DRIVE motor (1-21). | Wrong port: wrong motor spins, or "sensor lost". |
| `steerPort` | Smart port of the STEERING motor. | Same. |
| `driveReversed` | Make positive drive RPM move the robot forward on this module. Applied once, by PROS (a negative port number). | Wrong: that wheel drives backward. |
| `steerReversed` | Make positive steering motion mean clockwise. Applied once, by PROS. | Wrong: that pod turns the opposite way from its target, and runs away (never settles). |
| `position` | `Vec2` position from robot center. | Wrong or swapped: bad pinwheel in pure spin. |
| `couplingSign` | +1: a clockwise pod turn drags the wheel forward. Check per module with the wheel off the floor. | Wrong sign: the cancellation DOUBLES the wheel roll instead of removing it. |

---

## 4. Data types (types.hpp)

| Type | Fields | Meaning |
|---|---|---|
| `Vec2` | `xM`, `yM` | A position: meters right and forward of robot center. |
| `ChassisCommand` | `vxMps`, `vyMps`, `omegaRadPerSec` | What the whole robot should do: strafe speed (right positive), forward speed, and spin rate (clockwise positive). |
| `ModuleRequest` | `speedMps` (always 0 or more), `angleRad` | What kinematics asks of one module, before optimization. |
| `ModuleState` | `signedSpeedMps`, `angleRad`, `flipped`, `held` | After optimization. `signedSpeedMps` is NEGATIVE when flipped (the optimizer owns this sign; nothing downstream flips it again). `angleRad` is the pod target. |
| `ModuleTelemetry` | see below | One module's numbers for one cycle. |

`ModuleTelemetry` fields: `targetDeg` (pod target), `podDeg` (measured pod angle),
`errorDeg` (steering error), `alignScale` (cosine scale applied to wheel speed),
`requestedPodRpm` (profile rate), `actualPodRpm` (filtered measured rate),
`baseDriveRpm` (wheel term), `couplingDriveRpm` (cancellation term), `finalDriveRpm`
(sent to the motor), `steerMotorRpm` (sent to the steering motor), `lostBaseRpm` /
`lostCouplingRpm` (what saturation removed), `flipped`, `held`, `saturated`,
`sensorValid`. Only some of these are shown on screen (section 8).

---

## 5. Parameter structs

Each is built once from `config.hpp` and handed to the class that uses it. You
normally edit the constants, not these.

| Struct (file) | Fields |
|---|---|
| `InputParams` (`input_conditioner.hpp`) | `translationDeadband`, `rotationDeadband`, `linearWeight`, `cubicWeight`, `translationAccelPerSec`, `translationDecelPerSec`, `directionSlewRadPerSec`, `rotationAccelPerSec`, `rotationDecelPerSec`, `maxTranslationMps` (full stick speed), `maxOmegaRadPerSec` (full stick spin), `rotationSign`. Built by `config::makeInputParams()`. |
| `OptimizerParams` (`module_optimizer.hpp`) | `holdEnterMps`, `holdExitMps`, `flipEnterRad`, `flipExitRad`. |
| `SteeringProfileParams` (`steering_profile.hpp`) | `kpPerSec`, `maxRateRadPerSec`, `maxAccelRadPerSec2`, `settleToleranceRad`. |
| `DriveCompParams` (`drive_compensation.hpp`) | `motorRpmPerWheelMps`, `cancelMotorRevPerPodRev`, `couplingSign`, `requestedRateWeight`, `podRateDeadbandRadPerSec`, `driveLimitRpm`. |
| `ModuleControllerParams` (`module_controller.hpp`) | `optimizer`, `steering`, `drive` (the three above), `steerMotorRevPerPodRev`, `steerNominalRpm`, `actualRateFilterTauSec`. Built per module by `config::makeModuleControllerParams()`. |

---

## 6. Functions and classes by file

### angle_math.hpp (pure math)
| Name | What it does |
|---|---|
| `kPi`, `kTwoPi` | Constants. |
| `degToRad`, `radToDeg`, `rpmToRadPerSec`, `radPerSecToRpm` | Unit conversions. |
| `clampValue(v, lo, hi)` | Limit a number to a range. |
| `wrapPi(a)` | Wrap an angle into [-pi, pi). Returns 0 for infinity or NaN (an unplugged motor reads infinity), so a bad sensor cannot freeze the loop. |
| `shortestError(target, measured)` | Signed shortest turn from one angle to another; 359 to 1 is +2, not -358. |
| `compassDegrees(a)` | An angle as 0-360 compass degrees, for screens. |

### kinematics.hpp
| Name | What it does |
|---|---|
| `kNumModules` (4) | Module count; also sizes every array. |
| `KinematicsResult` | `modules` (a request per module), `largestRawMps` (fastest module before normalizing), `scale` (the one common factor applied, 1.0 if none). |
| `inverseKinematics(command, positions, maxWheelSpeedMps)` | Turns the chassis command into four module requests, then scales ALL modules by the same factor if any exceeds the limit (so the move keeps its shape). |

### input_conditioner.hpp
| Name | What it does |
|---|---|
| `RawInput` | `forward`, `strafe`, `rotate`: raw stick counts, -127..127. |
| `InputConditioner::update(raw, dt)` | The whole input chain: normalize, radial deadband, response curve, then slew limits, then output a `ChassisCommand`. |
| `reset()` | Zero the slew state (done at homing). |
| `translationMagnitude()` | Current normalized translation speed; used by the tests. |
| `radialDeadband`, `shape` | Deadband on a magnitude; the response curve. |
| `slewTranslation`, `slewRotation` | Rate limits. Translation limits magnitude and direction separately. |
| `kDirectionAdoptMagnitude` (0.02) | Below this speed the direction is simply adopted instead of slewed. |
| `kReversalAngleRad` (pi/2) | A requested direction change bigger than this slows the robot first. |

### module_optimizer.hpp
`ModuleOptimizer::update(request, measuredPodRad)` picks the pod target and the signed
wheel speed: low-speed hold (with enter/exit thresholds), then direct or flipped
(with hysteresis). `reset()` forgets state; `flipped()` / `held()` report state (used
by tests).

### steering_profile.hpp
`SteeringProfile::update(errorRad, dt)` returns the profiled pod RATE (rad/s):
proportional pull, capped at the max rate, with a braking limit `sqrt(2 * accel * |error|)`
so the pod slows in time, with the rate itself changing no faster than the max accel.
`reset()` zeroes it.

### drive_compensation.hpp
| Name | What it does |
|---|---|
| `alignmentScale(errorRad)` | Cosine of the steering error, never negative: full speed when aligned, zero at 90 degrees off. |
| `DriveSetpoint` | `baseRpm`, `couplingRpm`, `finalRpm`, `lostBaseRpm`, `lostCouplingRpm`, `effectivePodRateRadPerSec`, `saturated`. |
| `calculateDriveSetpoint(alignedWheelMps, requestedPodRate, actualPodRate, params)` | `final = base + coupling`, where base is the wheel request and coupling is the pod-rate cancellation. If the sum exceeds the limit, coupling wins and the base term is squeezed. |

### module_controller.hpp
`ModuleController::step(request, podContinuousRad, measuredPodRate, dt)` runs one
module's whole step with no hardware: optimizer, steering error, profile, cosine scale,
drive setpoint; returns `ModuleOutput` (`driveMotorRpm`, `steerMotorRpm`, telemetry).
`reset()` clears its state.

### swerve_module.hpp / .cpp (hardware, one module)
| Name | What it does |
|---|---|
| `SwerveModule(config)` | Creates the drive and steer motors with the right gearsets and reversal. |
| `configureMotors()` | Brake modes: drive coasts, steering brakes. |
| `tareSteering()` | Zero the steering encoder (pod must be centered). Returns false on failure. |
| `sample()` | Read the steering position and velocity once and convert to pod units; returns false if a sensor is invalid. |
| `compute(request, dt)` | Run the controller; no motor output yet. |
| `commit()` | Send both motor commands. |
| `stop()` | Zero both motors immediately. |
| `resetController()` | Forget optimizer and profile state (after homing). |
| `drivePowerWatts()` | Drive motor power, for the controller screen. |
| `telemetry()`, `name()` | Accessors. |
| `gearsetFor`, `signedPort` (file-local) | Cartridge to PROS gearset; negative port means reversed. |

### swerve_drive.hpp / .cpp (hardware, whole drivetrain)
| Name | What it does |
|---|---|
| `SwerveDrive` | The only owner of all eight motors. |
| `configure()` | Brake modes, then stop. |
| `homeSteering()` | Zero all four steering encoders. Sets `homed_`. |
| `periodic(raw, dt)` | One driver-control cycle: condition input, then run the cycle. |
| `driveChassis(command, dt)` | Same, but skips input conditioning (autonomous). |
| `runCycle` | Refuses to drive until homed; reads every sensor once; kinematics; computes all four; THEN commits all four. Any bad sensor stops everything. |
| `stop()` | Stop all motors. |
| `module(i)`, `snapshot()` | Access one module; copy the telemetry for the display task. |
| `DriveTelemetry` | `homed`, `sensorsOk`, `command`, `normalizationScale`, `dtSec`, `modules[4]`. |

### display.hpp / .cpp
| Name | What it does |
|---|---|
| `ControllerScreen` | Rate-limited controller text. `set(line, text)` stores, `service()` prints at most one line per 60 ms. |
| `kLines` (3), `kWidth` (15) | Controller screen size. |
| `kControllerMinGapMs` (60) | Minimum time between controller prints; the controller drops updates under about 50 ms. |
| `kControllerRefreshMs` (1000) | Unchanged lines are re-sent after this long, so a dropped print recovers. |
| `kBrainRefreshMs` (200) | Brain page refresh. |
| `brainPrint(line, text)` | Print one line on the brain. |
| `DisplayTask` | Low-priority task showing motor power on the controller and the diagnostics page on the brain. `start()` creates it (once); `setPaused()` lets the startup prompt own the screens. |

### startup.hpp / .cpp
`alignAndHome(controller, drive)`: prompts you to center the pods and press X, zeroes
the steering encoders, shows a confirmation, and returns. Drive stays off until it
returns. If zeroing fails it shows an error and stops.

### main.cpp
| Name | What it does |
|---|---|
| `measureDt(lastMs)` | Measured loop time in seconds, clamped to `kDtMinSec`..`kDtMaxSec`. |
| `autonomous()` | The 1 m calibration test: drives forward at a fixed speed (`speedMps`, 0.25 m/s or 70 percent of top speed, whichever is lower) for `durationMs` = `kDistanceM / speedMps`. Change `kDistanceM` for other distances. |
| `opcontrol()` | Homing prompt, then the 10 ms loop: read sticks, `drive.periodic()`. `master`, `drive`, `display` are `static` so they can never dangle when PROS ends the task. |
| `initialize`, `disabled`, `competition_initialize` | Brain screen messages only. |

---

## 7. Internal state variables

These hold information between cycles. You should not need to edit them.

| Variable | Class | Meaning |
|---|---|---|
| `p_` | most classes | A copy of that class's parameters. |
| `magnitude_` | `InputConditioner` | Current translation speed, 0..1 after slewing. |
| `directionRad_` | `InputConditioner` | Current translation direction after slewing. |
| `omega_` | `InputConditioner` | Current spin, -1..1 after slewing. |
| `initialized_` | `ModuleOptimizer` | False until the first cycle after (re)homing, when it adopts the pod's current angle. |
| `held_` | `ModuleOptimizer` | Angle hold active (wheel stopped, angle kept). |
| `flipped_` | `ModuleOptimizer` | This module is in the flipped state (pod at the opposite angle, wheel reversed). |
| `lastTargetRad_` | `ModuleOptimizer` | Last real pod target, kept while held. |
| `rateRadPerSec_` | `SteeringProfile` | The current profiled pod rate. |
| `primed_` | `ModuleController` | False until the first measurement seeds the rate filter (avoids a startup transient). |
| `actualPodRate_` | `ModuleController` | Filtered measured pod rate. |
| `config_`, `drive_`, `steer_`, `controller_` | `SwerveModule` | This module's config, motors and pure controller. |
| `podContinuousRad_`, `podRateRadPerSec_` | `SwerveModule` | Latest pod angle (may exceed one turn) and pod rate from the steering encoder. |
| `sensorValid_` | `SwerveModule` | Latest sample was finite and usable. |
| `output_`, `telemetry_` | `SwerveModule` | Latest computed motor commands and diagnostics. |
| `modules_`, `positions_`, `conditioner_` | `SwerveDrive` | The four modules, their positions, the input chain. |
| `homed_` | `SwerveDrive` | The steering encoders have a known zero. Drive is off while false. |
| `telemetry_` | `SwerveDrive` | Snapshot for the display task. |
| `lines_`, `lastAnyPrintMs_`, `next_` | `ControllerScreen` | Pending text, time of last print, which line goes next. |
| `paused_` | `DisplayTask` | While true the display task prints nothing. |
| `task_` | `DisplayTask` | The background task. |

---

## 8. Reading the screens

**Brain screen (updates 5 times a second):**

```
SWERVE v2  OK                       status: OK / NOT HOMED / SENSOR FAIL
vx+0.00 vy+0.35 w+0.00 m/s,rad/s    the conditioned chassis command
scale 1.00  dt 10.0 ms              kinematics scale (below 1 = speed limited), loop time
FL t 45 p 44 d+120 c -10 F-S        one line per module
```

Per-module letters: `t` pod target angle (degrees, compass), `p` measured pod angle,
`d` final drive RPM, `c` the coupling (cancellation) part of that RPM. Flags:
`F` flipped, `H` held, `S` saturated. A dash means off.

**Controller screen:** line 1 is drive power in watts for the front pair, line 2 for the
rear pair, line 3 is the status. A motor at its limit shows a high, flat wattage.

**What the readings tell you:**
- `p` far from `t` for a long time: the pod is not reaching its target (steering sign,
  gain, or a mechanical problem).
- `c` large while the stick is at rest: the pods are turning and cancelling drag.
- `S` showing in normal driving: lower `kDriveUsableFraction`, or reduce the pod rate.
- `scale` under 1.00 during combined driving: the commands exceed what the wheels can do
  and everything is being scaled together (try lowering `kRotationAuthority`).

---

## 9. Symptom to knob cheat sheet

| Symptom | First things to try |
|---|---|
| Robot creeps while pods turn | `couplingSign` per module, then `kInducedWheelRevPerPodRev`, `kDriveMotorRevPerWheelRev`, `kRequestedRateWeight` |
| Wheels spin the wrong way while pods turn (worse, not better) | `couplingSign` is wrong for that module, or the cancellation constant is far too large |
| Pods hunt or buzz near their target | Lower `kPodRateKpPerSec`; raise `kPodSettleToleranceDeg` |
| Pods lag the stick | Raise `kPodRateKpPerSec`, `kPodMaxRateDegPerSec`, `kPodMaxAccelDegPerSec2` |
| Pod runs away and never settles | That module's `steerReversed` is wrong |
| A wheel drives backward | That module's `driveReversed` is wrong |
| Pods point at each other during pure spin | Module positions or ports swapped: README step 5 decoder |
| Spins the wrong way but the pinwheel is right | `kRotationSign` |
| Drives a different distance than commanded | `kDriveMotorRevPerWheelRev`, then `kWheelDiameterM` |
| Pods flip back and forth near sideways | Widen the gap between `kFlipEnterDeg` and `kFlipExitDeg` |
| Pods twitch at very low stick speeds | Raise `kHoldEnterFraction` and `kHoldExitFraction` |
| Robot cuts out at low speed | Lower the hold fractions |
| Too twitchy or sensitive near center | More cubic weight, bigger deadband |
| Takes off too hard / tips | Lower `kTranslationAccelPerSec` |
| Slow to stop | Raise `kTranslationDecelPerSec` |
| Spin overpowers driving when both sticks are used | Lower `kRotationAuthority` |
| Runs out of speed when the battery drops | Lower `kDriveUsableFraction` (0.9) |
| Everything is slower than expected | Check `kBaseDriveLimitRpm` and `kReserveMargin`; a faster drive cartridge helps |
