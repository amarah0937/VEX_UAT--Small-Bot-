#include "main.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <iostream>

#include "drivetrain/angle_math.hpp"
#include "drivetrain/chassis_command.hpp"
#include "drivetrain/drivetrain_config.hpp"

namespace {

namespace cfg = drivetrain::config;

constexpr int kScreenRefreshMs = 100;
constexpr std::size_t kNumModules = 4;
// Order matters: this is the order modules are stored and printed in.
constexpr std::array<const char*, kNumModules> kModuleNames = {"FL", "RL", "FR", "RR"};

// True once the steer encoders have been zeroed with the pods centered.
// opcontrol() sets it (press X); autonomous() zeroes them itself if it is
// still false, which assumes the pods were centered by hand before the match.
bool g_steerEncodersZeroed = false;

// Per-module memory that lasts between loop cycles.
struct ModuleControlState {
	double lastTargetHeadingRad = 0.0;  // last heading we commanded (reverse decision is made against this)
	bool hasTargetHeading = false;
	double filteredSteerRpm = 0.0;      // low-passed steer velocity used by the coupling compensation
};

constexpr std::int8_t signedPort(const std::int8_t port, const bool reversed) {
	return static_cast<std::int8_t>(reversed ? -port : port);
}

// CHANGED: one struct per module instead of 8 separate motors, 4 control states
// and 4 copies of every line of code. Behavior is the same, but a fix now only
// has to be made once.
struct SwerveModule {
	explicit SwerveModule(const cfg::ModuleConfig& module_config)
		: config(module_config),
		  drive(signedPort(module_config.drivePort, module_config.driveReversed),
				pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees),
		  steer(signedPort(module_config.steerPort, module_config.steerReversed),
				pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees) {}

	cfg::ModuleConfig config;
	pros::Motor drive;
	pros::Motor steer;
	ModuleControlState control;
};

// Everything we compute for one module in one loop cycle.
struct ModuleCycle {
	double steerPositionDeg = 0.0;  // motor-side encoder position
	double currentAngleRad = 0.0;   // pod angle derived from that position
	double encoderTarget = 0.0;     // motor-side encoder target for move_absolute
	drivetrain::ModuleState state;  // optimized speed + heading
	int driveRpm = 0;               // final drive motor command
};

// Controller screen: 3 lines x 15 characters, and the controller only accepts
// a new text update about every 50 ms.
//
// CHANGED: this used to print two lines back to back with a pros::delay(60)
// between them, which froze the whole control loop for 60 ms every 100 ms. It
// also printed 17 characters per line (the limit is 15), and the next update
// arrived ~40 ms after the previous one (inside the 50 ms window), so some
// updates were dropped. That is why only one line seemed to update.
//
// Now: no delay, one line per call, each line 13 characters. Call this every
// 3rd control cycle (60 ms). Calls alternate between the two lines, so each
// line refreshes every 120 ms.
//   line 0:  FL 5.2 FR 4.9   (front pair, power in watts)
//   line 1:  RL 5.0 RR 5.1   (rear pair)
// Modules are stored in the order FL, RL, FR, RR.
void updateControllerDisplay(pros::Controller& controller,
							 std::array<SwerveModule, kNumModules>& modules,
							 const bool show_front_row) {
	// get_power() returns the motor's output power in watts (about 0-11 W).
	// REMOVED: the separate global FL/RL/FR/RR pros::Motor objects. They sat on
	// the same ports as the drive motors and hard-coded the port numbers a
	// second time; reading the SwerveModule drive motors avoids that.
	const double fl = modules[0].drive.get_power();
	const double rl = modules[1].drive.get_power();
	const double fr = modules[2].drive.get_power();
	const double rr = modules[3].drive.get_power();

	if (show_front_row) {
		controller.print(0, 0, "FL%4.1f FR%4.1f", fl, fr);
	} else {
		controller.print(1, 0, "RL%4.1f RR%4.1f", rl, rr);
	}
	
}

// ---- Module motion telemetry (terminal) --------------------------------------
constexpr bool kLogModuleMotion = true;     // false turns the module printout off
// CHANGED: 10 -> 25 (a printout every 500 ms). At 200 ms the four long module
// lines plus the GPS lines overran the serial link and the terminal showed
// "Could not decode bytes" warnings.
constexpr int kModuleLogEveryCycles = 25;   // 25 x 20 ms = a printout every 500 ms
int g_moduleLogCounter = 0;

// Prints the motion of all four modules to the terminal, one line each:
//   pod / podTarget : pod (steering) angle now and where it is being sent, in
//                     compass degrees (0 = forward, clockwise positive)
//   steerPos / steerTarget : steer motor encoder position and move_absolute
//                     target, in motor degrees
//   steerVel        : measured steer motor velocity (rpm)
//   drivePos / driveVel : drive motor encoder position (degrees) and measured
//                     velocity (rpm)
//   driveCmd        : the velocity command sent to the drive motor this cycle
//                     (rpm, already clamped to kDriveMotorMaxVelocityRpm)
void logModuleMotion(std::array<SwerveModule, kNumModules>& modules,
					 const std::array<ModuleCycle, kNumModules>& cycle,
					 const char* label) {
	std::cout << "[MODULES " << label << "] t=" << pros::millis() << " ms\n";
	for (std::size_t i = 0; i < kNumModules; ++i) {
		auto& module = modules[i];
		const auto& c = cycle[i];
		std::cout << "  " << kModuleNames[i]
				  << " pod=" << drivetrain::compassDegrees(c.currentAngleRad)
				  << " podTarget=" << drivetrain::compassDegrees(c.state.angleRad)
				  << " steerPos=" << c.steerPositionDeg
				  << " steerTarget=" << c.encoderTarget
				  << " steerVel=" << module.steer.get_actual_velocity()
				  << " | drivePos=" << module.drive.get_position()
				  << " driveVel=" << module.drive.get_actual_velocity()
				  << " driveCmd=" << c.driveRpm
				  << " (max " << cfg::kDriveMotorMaxVelocityRpm << ")\n";
	}
	std::cout << std::flush;
}

// Same, but only every kModuleLogEveryCycles calls so the loop isn't slowed.
void maybeLogModuleMotion(std::array<SwerveModule, kNumModules>& modules,
						  const std::array<ModuleCycle, kNumModules>& cycle,
						  const char* label) {
	if (!kLogModuleMotion) return;
	if (g_moduleLogCounter++ % kModuleLogEveryCycles == 0) {
		logModuleMotion(modules, cycle, label);
	}
}

// Joystick counts (-127..127) -> -1..1 with a deadband that starts from zero
// (no jump when the stick leaves the deadband).
double normalizedAxis(const int value) {
	const int magnitude = std::abs(value);
	if (magnitude <= cfg::kJoystickDeadband) return 0.0;
	const double scaled = static_cast<double>(magnitude - cfg::kJoystickDeadband) /
						  (127.0 - cfg::kJoystickDeadband);
	return std::copysign(scaled, static_cast<double>(value));
}

// Inverse kinematics for one module: chassis command -> speed + heading.
//
// CHANGED: removed the current_angle parameter. It was only returned when the
// speed was ~0, and optimizeModuleState() overwrote that case anyway.
//
// CHANGED: the rotation mirroring (kMirrorRotationVectorX) is applied to the
// rotation vector here, before it is added to translation. It replaces the old
// "if (pure_rotation) angle = -angle" patch in the main loop, which only ran
// when the rotation stick was used alone.
drivetrain::ModuleState calculateModuleState(
	const drivetrain::ChassisCommand& command,
	const drivetrain::ModulePosition& position,
	const bool chassis_stopped) {
	if (chassis_stopped) return {0.0, 0.0};

	const double rotation_x_sign = cfg::kMirrorRotationVectorX ? -1.0 : 1.0;
	const double module_vx = command.vx + rotation_x_sign * command.omega * position.yIn;
	const double module_vy = command.vy - command.omega * position.xIn;
	return {std::hypot(module_vx, module_vy), std::atan2(module_vx, module_vy)};
}

// Motor-side encoder target (degrees) that points the pod at target_angle_rad
// by the shortest route, without unwinding extra full turns.
//
// CHANGED: takes the position as a number instead of a pros::Motor. It used to
// call motor.get_position() itself, so a single loop cycle read each encoder
// three times at slightly different moments. Now it is read once per cycle.
double closestSteerEncoderTargetDeg(const double steer_position_deg,
									const double target_angle_rad) {
	const double current_module_angle_rad = drivetrain::degToRad(
		steer_position_deg / cfg::kSteerMotorRevPerModuleRev);
	const double angle_error_rad = drivetrain::shortestAngularError(
		target_angle_rad, current_module_angle_rad);
	return steer_position_deg +
		   drivetrain::radToDeg(angle_error_rad) * cfg::kSteerMotorRevPerModuleRev;
}

// Picks the pod heading and the wheel direction for one module.
//
// CHANGED (pod range): the pod axis is kept in the FRONT half-circle
// (-90..+90 degrees, plus kPodRangeOvershootRad of hysteresis) and the wheel
// is reversed for anything that points backward. So forward <-> backward only
// ever reverses the wheel; the pod never turns 180 degrees. Inside the range
// the pod still takes the shortest route, so sweeping the stick keeps the pod
// moving smoothly. The only time the pod flips by ~180 is when the requested
// heading crosses the pure-strafe line far enough to leave the range.
//
// CHANGED (parking): the old "park at 0 or 180 when sticks are released" snap
// is gone. Releasing the sticks just holds the current heading with the wheel
// stopped, so nothing turns until the next command (and the neutral-heading
// bookkeeping and its hysteresis are no longer needed).
drivetrain::ModuleState optimizeModuleState(
	const drivetrain::ModuleState& requested_state,
	const double steer_position_deg,
	const double current_angle_rad,
	const bool chassis_stopped,
	ModuleControlState& control_state,
	double& encoder_target) {
	// The reverse decision is made against the last commanded heading, not the
	// measured one, so it stays stable while a pod is still catching up.
	const double reference_angle = control_state.hasTargetHeading
									   ? control_state.lastTargetHeadingRad
									   : drivetrain::wrapRadians(current_angle_rad);

	// Sticks released, or this module's speed is ~0 (near the instant center of
	// rotation): hold the heading, wheel stopped.
	if (chassis_stopped || requested_state.speed < cfg::kMinModuleSpeedInPerSec) {
		encoder_target = closestSteerEncoderTargetDeg(steer_position_deg, reference_angle);
		control_state.lastTargetHeadingRad = reference_angle;
		control_state.hasTargetHeading = true;
		return {0.0, reference_angle};
	}

	const double direct_angle = drivetrain::wrapRadians(requested_state.angleRad);
	const double reversed_angle =
		drivetrain::wrapRadians(requested_state.angleRad + drivetrain::kPi);
	const double direct_error =
		drivetrain::shortestAngularError(direct_angle, reference_angle);
	const double reversed_error =
		drivetrain::shortestAngularError(reversed_angle, reference_angle);

	// Shortest route first...
	bool reverse_drive = std::abs(reversed_error) < std::abs(direct_error);
	double selected_angle = reverse_drive ? reversed_angle : direct_angle;

	// ...unless that would put the pod axis too far into the back half. Exactly
	// one of the two candidates is in the front half, so the swap always fits.
	const double max_pod_angle = drivetrain::kPi / 2.0 + cfg::kPodRangeOvershootRad;
	if (std::abs(selected_angle) > max_pod_angle) {
		reverse_drive = !reverse_drive;
		selected_angle = reverse_drive ? reversed_angle : direct_angle;
	}

	encoder_target = closestSteerEncoderTargetDeg(steer_position_deg, selected_angle);
	control_state.lastTargetHeadingRad = selected_angle;
	control_state.hasTargetHeading = true;

	return {reverse_drive ? -requested_state.speed : requested_state.speed,
			selected_angle};
}

void stopAllMotors(std::array<SwerveModule, kNumModules>& modules) {
	for (auto& module : modules) {
		module.drive.move_velocity(0);
		module.steer.move(0);
	}
}

// Draws up to three lines on the brain screen. clear_first=false just redraws
// the same text (no erase, so no flicker).
void drawBrainMessage(const char* line1, const char* line2, const char* line3,
					  const bool clear_first) {
	if (clear_first) pros::screen::erase();
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "%s", line1);
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "%s", line2);
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "%s", line3);
}

// Controller text: 15 characters per line, ~50 ms between updates. Blocking is
// fine here because this only runs during startup, never in the control loop.
void drawControllerMessage(pros::Controller& controller, const char* line0,
						   const char* line1) {
	controller.clear();
	pros::delay(60);
	controller.print(0, 0, "%s", line0);
	pros::delay(60);
	controller.print(1, 0, "%s", line1);
	pros::delay(60);
}

// Startup: the driver centers the pods by hand, presses X on the controller,
// and the steer encoders are zeroed there. Returns once that is done.
//
// CHANGED: moved out of opcontrol() into this helper. The prompt is redrawn on
// the brain every 100 ms while waiting (so it cannot stay blank), it is also
// shown on the controller screen where you are pressing X, pen/eraser colors are
// set explicitly, and the confirmation is held for 1.5 s before returning.
void waitForPodAlignmentAndTare(pros::Controller& controller,
								std::array<SwerveModule, kNumModules>& modules) {
	constexpr int kRedrawEveryPasses = 5;  // 5 passes x 20 ms = 100 ms
	pros::screen::set_pen(pros::Color::white);
	pros::screen::set_eraser(pros::Color::black);

	const char* prompt1 = "Mode: opcontrol";
	const char* prompt2 = "Center all four pods";
	const char* prompt3 = "Press X to zero encoders";
	drawBrainMessage(prompt1, prompt2, prompt3, true);
	drawControllerMessage(controller, "Center pods", "Press X to zero");

	// If X is still held from before, wait for release so it can't exit at once.
	while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
		pros::delay(cfg::kControlPeriodMs);
	}
	// Wait for a fresh press, redrawing the prompt as we go.
	int pass = 0;
	while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
		if (++pass % kRedrawEveryPasses == 0) drawBrainMessage(prompt1, prompt2, prompt3, false);
		pros::delay(cfg::kControlPeriodMs);
	}

	// Taring makes "pods straight" the zero position of every steer encoder.
	bool tare_failed = false;
	for (auto& module : modules) {
		// tare_position() reports failure as PROS_ERR (INT32_MAX), not a negative number.
		if (module.steer.tare_position() == PROS_ERR) tare_failed = true;
	}
	if (tare_failed) {
		stopAllMotors(modules);
		drawBrainMessage("ERROR: encoder tare failed", "Outputs remain stopped",
						 "Restart and check motors", true);
		drawControllerMessage(controller, "TARE FAILED", "Restart robot");
		while (true) {
			drawBrainMessage("ERROR: encoder tare failed", "Outputs remain stopped",
							 "Restart and check motors", false);
			pros::delay(100);
		}
	}

	g_steerEncodersZeroed = true;
	drawBrainMessage(prompt1, "Encoder zero established", "Four-pod control active", true);
	drawControllerMessage(controller, "Encoders zeroed", "Control active");
	for (int i = 0; i < 15; ++i) {  // hold for 1.5 s, redrawing
		drawBrainMessage(prompt1, "Encoder zero established", "Four-pod control active", false);
		pros::delay(100);
	}
}

// =============================================================================
// Autonomous helpers
// =============================================================================

// ---- Tunables for the autonomous routine ------------------------------------
constexpr double kInchesPerMeter = 39.3701;
constexpr double kAutonDistanceIn = 1.0 * kInchesPerMeter;  // each 1 m leg
constexpr double kAutonSpeedFraction = 0.3;         // of kMaxLinearSpeedInPerSec
constexpr double kAutonMinApproachFraction = 0.25;  // slowest creep near the end
constexpr double kAutonSlowZoneIn = 6.0;            // start slowing this far from the goal
constexpr double kAutonSettleMs = 2000;             // pause between legs

// Heading turn (absolute GPS heading, 0 = the way the robot faced at startup,
// increasing clockwise). Max spin is kAutonTurnFraction * 540 deg/s (about
// 65 deg/s at 0.12) and it eases down to kAutonMinTurnFraction (about 16 deg/s)
// inside the slow zone, so it can land inside +-0.5 deg without overshooting.
constexpr double kAutonTargetHeadingDeg = 90.0;
constexpr double kAutonHeadingToleranceDeg = 0.5;
constexpr double kAutonTurnFraction = 0.12;
constexpr double kAutonMinTurnFraction = 0.03;
constexpr double kAutonTurnSlowZoneDeg = 30.0;
// Must stay inside the tolerance this many cycles in a row (10 x 20 ms) before
// the turn counts as done, so one lucky GPS reading doesn't end it early.
constexpr int kAutonHeadingSettleCycles = 10;
// The turn keeps going until it is on target; this only stops it if something
// is badly wrong (wrong spin direction, GPS dead) so it can't spin forever.
constexpr std::uint32_t kAutonTurnSafetyTimeoutMs = 15000;

// Direction of the last leg in the ROBOT frame after the turn. +x is
// robot-right. After turning clockwise the robot's right side faces the
// field's original "backwards", so +1 here means "back toward the start line,
// sideways". Use -1 to go the other way.
constexpr double kFinalLegRobotVx = 1.0;

// GPS sensor setup (meters / degrees). Offset = where the sensor sits on the
// robot; initial = where the robot starts on the field.
constexpr std::int8_t kGpsPort = 4;
constexpr double kGpsXOffset = 0.0;
constexpr double kGpsYOffset = 0.0;
constexpr double kGpsInitialX = -1.6;
constexpr double kGpsInitialY = -0.1;
constexpr double kGpsInitialHeadingDeg = 88.6;
constexpr std::uint32_t kGpsDataRateMs = 5;
constexpr double kGpsMaxErrorM = 0.2;           // reported accuracy must be at least this good
// CHANGED: replaces kGpsMaxJumpM (0.01 m between samples). The GPS position
// wobbles by far more than that even when the robot is standing still (the log
// showed y moving 0.1-0.4 per sample), so that test could never pass. Stable
// now means: reported error is good AND the heading stays within this many
// degrees of where the stable run started.
constexpr double kGpsMaxHeadingDriftDeg = 0.5;
constexpr int kGpsStableSamples = 15;            // for this many samples in a row
constexpr std::uint32_t kGpsStabilizeTimeoutMs = 4000;
constexpr int kGpsLogEveryCycles = 5;            // terminal print every 100 ms

void autonStatus(const char* line2, const char* line3 = "") {
	pros::screen::erase();
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "Mode: autonomous");
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "%s", line2);
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "%s", line3);
}

// Prints the GPS status to the terminal (x/y in meters, heading in degrees,
// error = the sensor's own accuracy estimate in meters).
void logGps(pros::GPS& gps, const char* label) {
	std::cout << "[GPS " << label << "]"
			  << " x=" << gps.get_position_x()
			  << " y=" << gps.get_position_y()
			  << " heading=" << gps.get_heading()
			  << " error=" << gps.get_error()
			  << std::endl;
}

// One control cycle of the same pipeline opcontrol() uses (kinematics ->
// pod optimizing -> global speed scale -> alignment slow-down -> steer
// coupling compensation -> motor outputs), driven by a ChassisCommand instead
// of the joysticks. This is the drivetrain's holonomic function: vx/vy are
// translation in in/s (+x right, +y forward) and omega is rotation in rad/s,
// all at the same time if you want.
//
// Returns false (and stops all motors) if a steer sensor is lost.
bool driveChassis(std::array<SwerveModule, kNumModules>& modules,
				  const drivetrain::ChassisCommand& command) {
	const bool chassis_stopped =
		command.vx == 0.0 && command.vy == 0.0 && command.omega == 0.0;

	std::array<ModuleCycle, kNumModules> cycle{};
	for (std::size_t i = 0; i < kNumModules; ++i) {
		cycle[i].steerPositionDeg = modules[i].steer.get_position();
		if (!std::isfinite(cycle[i].steerPositionDeg)) {
			stopAllMotors(modules);
			return false;
		}
	}

	for (std::size_t i = 0; i < kNumModules; ++i) {
		auto& module = modules[i];
		auto& c = cycle[i];
		c.currentAngleRad = drivetrain::degToRad(
			c.steerPositionDeg / cfg::kSteerMotorRevPerModuleRev);
		const drivetrain::ModuleState requested = calculateModuleState(
			command, module.config.position, chassis_stopped);
		c.state = optimizeModuleState(requested, c.steerPositionDeg,
									  c.currentAngleRad, chassis_stopped,
									  module.control, c.encoderTarget);

		const double measured_rpm = module.steer.get_actual_velocity();
		if (std::isfinite(measured_rpm)) {
			module.control.filteredSteerRpm +=
				cfg::kSteerVelocityFilterAlpha *
				(measured_rpm - module.control.filteredSteerRpm);
		}
	}

	double largest_module_speed = 0.0;
	for (const auto& c : cycle) {
		largest_module_speed = std::max(largest_module_speed, std::abs(c.state.speed));
	}
	const double speed_scale =
		largest_module_speed > cfg::kMaxLinearSpeedInPerSec
			? cfg::kMaxLinearSpeedInPerSec / largest_module_speed
			: 1.0;

	const double wheel_circumference_in = drivetrain::kPi * cfg::kWheelDiameterIn;
	for (std::size_t i = 0; i < kNumModules; ++i) {
		auto& module = modules[i];
		auto& c = cycle[i];

		const double steering_error = drivetrain::shortestAngularError(
			c.state.angleRad, c.currentAngleRad);
		const double alignment_scale = std::max(0.0, std::cos(steering_error));
		const double wheel_rpm = c.state.speed * alignment_scale * speed_scale /
								 wheel_circumference_in * 60.0 *
								 cfg::kDriveMotorRevPerWheelRev;

		double steer_rpm = module.control.filteredSteerRpm;
		if (std::abs(steer_rpm) < cfg::kSteerVelocityDeadbandRpm) steer_rpm = 0.0;
		const double pod_rpm = steer_rpm / cfg::kSteerMotorRevPerModuleRev;
		const double steering_induced_drive_rpm =
			pod_rpm * cfg::kWheelRevPerPodRev * cfg::kDriveMotorRevPerWheelRev *
			cfg::kSteerCouplingCompensationGain;

		const double command_rpm = std::clamp(
			wheel_rpm - steering_induced_drive_rpm,
			-cfg::kDriveMotorMaxVelocityRpm, cfg::kDriveMotorMaxVelocityRpm);
		c.driveRpm = static_cast<int>(std::lround(command_rpm));
	}

	for (std::size_t i = 0; i < kNumModules; ++i) {
		modules[i].drive.move_velocity(cycle[i].driveRpm);
		modules[i].steer.move_absolute(cycle[i].encoderTarget,
									   cfg::kSteerTargetVelocityRpm);
	}
	maybeLogModuleMotion(modules, cycle, "auto");
	return true;
}

// Holds still (wheels stopped, pods keep their heading) for duration_ms.
bool holdStill(std::array<SwerveModule, kNumModules>& modules,
			   const std::uint32_t duration_ms, std::uint32_t& loop_start_ms) {
	const std::uint32_t end_ms = pros::millis() + duration_ms;
	while (pros::millis() < end_ms) {
		if (!driveChassis(modules, {0.0, 0.0, 0.0})) return false;
		pros::Task::delay_until(&loop_start_ms, cfg::kControlPeriodMs);
	}
	return true;
}

// Average distance (inches) the wheels have rolled since start_positions was
// recorded. Uses the drive motor encoders, so it works for any direction.
double wheelTravelInches(std::array<SwerveModule, kNumModules>& modules,
						 const std::array<double, kNumModules>& start_positions) {
	const double wheel_circumference_in = drivetrain::kPi * cfg::kWheelDiameterIn;
	double total = 0.0;
	for (std::size_t i = 0; i < kNumModules; ++i) {
		const double delta_deg = std::abs(modules[i].drive.get_position() - start_positions[i]);
		total += delta_deg / 360.0 / cfg::kDriveMotorRevPerWheelRev * wheel_circumference_in;
	}
	return total / static_cast<double>(kNumModules);
}

// Translate distance_in inches along the unit direction (dir_x, dir_y) in the
// robot frame (+x right, +y forward), with no rotation. Slows down near the
// end. Returns false on a sensor failure or timeout (motors are stopped).
bool driveDistance(std::array<SwerveModule, kNumModules>& modules,
				   const double dir_x, const double dir_y,
				   const double distance_in, std::uint32_t& loop_start_ms) {
	std::array<double, kNumModules> start_positions{};
	for (std::size_t i = 0; i < kNumModules; ++i) {
		start_positions[i] = modules[i].drive.get_position();
	}

	const double cruise_in_per_sec = kAutonSpeedFraction * cfg::kMaxLinearSpeedInPerSec;
	// Generous timeout: twice the ideal time plus a second for pods to turn.
	const std::uint32_t timeout_ms =
		static_cast<std::uint32_t>(2000.0 * distance_in / cruise_in_per_sec) + 1000;
	const std::uint32_t start_ms = pros::millis();

	while (true) {
		const double remaining_in = distance_in - wheelTravelInches(modules, start_positions);
		if (remaining_in <= 0.0) break;
		if (pros::millis() - start_ms > timeout_ms) {
			stopAllMotors(modules);
			return false;
		}

		const double approach_fraction = std::clamp(
			remaining_in / kAutonSlowZoneIn, kAutonMinApproachFraction, 1.0);
		const double speed = cruise_in_per_sec * approach_fraction;
		if (!driveChassis(modules, {dir_x * speed, dir_y * speed, 0.0})) return false;
		pros::Task::delay_until(&loop_start_ms, cfg::kControlPeriodMs);
	}
	return holdStill(modules, static_cast<std::uint32_t>(kAutonSettleMs), loop_start_ms);
}

// Waits (robot held still) until the GPS has been healthy and steady for
// kGpsStableSamples samples in a row, printing its status to the terminal as it
// goes. A sample is healthy when the sensor's own error estimate is within
// kGpsMaxErrorM; the run stays steady while the heading stays within
// kGpsMaxHeadingDriftDeg of where the run started (wrap-safe near 0/360).
// Position is deliberately NOT tested: it wobbles too much sample to sample.
// Returns true if it settled, false on timeout.
bool stabilizeGps(pros::GPS& gps, std::array<SwerveModule, kNumModules>& modules,
				  std::uint32_t& loop_start_ms) {
	const std::uint32_t start_ms = pros::millis();
	int stable_samples = 0;
	int cycle = 0;
	double reference_heading = 0.0;  // heading at the start of the current stable run

	while (pros::millis() - start_ms < kGpsStabilizeTimeoutMs) {
		if (!driveChassis(modules, {0.0, 0.0, 0.0})) return false;
		pros::Task::delay_until(&loop_start_ms, cfg::kControlPeriodMs);

		const double error_m = gps.get_error();
		const double heading = gps.get_heading();

		if (!std::isfinite(error_m) || !std::isfinite(heading) || error_m > kGpsMaxErrorM) {
			stable_samples = 0;
		} else {
			if (stable_samples == 0) reference_heading = heading;
			// Wrap-safe difference in [-180, 180).
			const double drift =
				std::fmod(heading - reference_heading + 540.0, 360.0) - 180.0;
			if (std::abs(drift) > kGpsMaxHeadingDriftDeg) {
				reference_heading = heading;  // start a new run from here
				stable_samples = 1;
			} else {
				++stable_samples;
			}
		}

		if (cycle++ % kGpsLogEveryCycles == 0) {
			logGps(gps, "stabilizing");
			std::cout << "[GPS stabilizing] stable " << stable_samples << "/"
					  << kGpsStableSamples << std::endl;
		}
		if (stable_samples >= kGpsStableSamples) {
			logGps(gps, "settled");
			return true;
		}
	}
	logGps(gps, "TIMEOUT");
	return false;
}

double wrapDegrees180(double degrees) {
	while (degrees > 180.0) degrees -= 360.0;
	while (degrees <= -180.0) degrees += 360.0;
	return degrees;
}

// Rotates in place until the GPS heading is within +-kAutonHeadingToleranceDeg
// of target_deg and has stayed there for kAutonHeadingSettleCycles in a row.
// It picks the shorter direction, slows down as it closes in, and corrects
// itself if it overshoots. Returns false if the GPS heading is invalid or the
// safety timeout expires (motors are stopped).
bool turnToGpsHeading(pros::GPS& gps, std::array<SwerveModule, kNumModules>& modules,
					  const double target_deg, std::uint32_t& loop_start_ms) {
	const std::uint32_t start_ms = pros::millis();
	int in_tolerance_cycles = 0;
	int cycle = 0;

	while (true) {
		const double heading = gps.get_heading();
		if (!std::isfinite(heading)) {
			stopAllMotors(modules);
			return false;
		}
		if (pros::millis() - start_ms > kAutonTurnSafetyTimeoutMs) {
			stopAllMotors(modules);
			logGps(gps, "turn TIMEOUT");
			return false;
		}

		// Positive error = turn clockwise, negative = counterclockwise.
		const double error_deg = wrapDegrees180(target_deg - heading);

		if (cycle++ % kGpsLogEveryCycles == 0) {
			logGps(gps, "turning");
			std::cout << "[GPS turning] target=" << target_deg
					  << " error=" << error_deg << " deg" << std::endl;
		}

		if (std::abs(error_deg) <= kAutonHeadingToleranceDeg) {
			// On target: hold still and make sure it stays there.
			if (++in_tolerance_cycles >= kAutonHeadingSettleCycles) break;
			if (!driveChassis(modules, {0.0, 0.0, 0.0})) return false;
		} else {
			in_tolerance_cycles = 0;
			const double fraction = kAutonTurnFraction *
				std::clamp(std::abs(error_deg) / kAutonTurnSlowZoneDeg,
						   kAutonMinTurnFraction / kAutonTurnFraction, 1.0);
			const double direction = error_deg > 0.0 ? 1.0 : -1.0;
			// Same sign convention as opcontrol(): omega * kRotationSign.
			const double omega = direction * fraction *
								 cfg::kMaxAngularSpeedRadPerSec * cfg::kRotationSign;
			if (!driveChassis(modules, {0.0, 0.0, omega})) return false;
		}
		pros::Task::delay_until(&loop_start_ms, cfg::kControlPeriodMs);
	}

	logGps(gps, "turn done");
	return holdStill(modules, static_cast<std::uint32_t>(kAutonSettleMs), loop_start_ms);
}

}  // namespace

// REMOVED: on_center_button() and every pros::lcd call. LLEMU and
// pros::screen both draw on the brain screen and overwrite each other, and the
// button callback was never registered anyway. Everything now uses pros::screen.

/**
 * Runs initialization code. This occurs as soon as the program is started.
 */
void initialize() {
	pros::screen::set_pen(pros::Color::white);
	pros::screen::erase();
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "DriftPod v2");
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "Initializing");
}

/**
 * Runs while the robot is disabled. No motors are commanded here.
 */
void disabled() {
	pros::screen::erase();
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "Mode: disabled");
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "Drivetrain outputs off");
}

/**
 * Runs before autonomous when connected to field control.
 */
void competition_initialize() {
	pros::screen::erase();
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "Mode: competition initialize");
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "Center pods before start");
}

/**
 * Runs the user autonomous code.
 *
 * Routine:
 *   1. Drive forward 1 m, then rotate back to a GPS heading of 0 (+-0.5 deg).
 *   2. Hold still until the GPS settles (accuracy good, position steady).
 *   3. Rotate to a GPS heading of 90 degrees (+-0.5 deg, slow and closed loop).
 *   4. Strafe 1 m sideways, which is "backwards" relative to where the robot
 *      started (see kFinalLegRobotVx), using the holonomic driveChassis().
 *
 * GPS status is printed to the terminal throughout. The pods must be centered
 * (pointing forward) when autonomous starts. If opcontrol() has not zeroed the
 * steer encoders yet, they are zeroed here. Any failure (lost sensor, GPS never
 * settles, timeout) stops the motors and ends the routine.
 */
void autonomous() {
	std::array<SwerveModule, kNumModules> modules{{
		SwerveModule(cfg::kFL), SwerveModule(cfg::kRL),
		SwerveModule(cfg::kFR), SwerveModule(cfg::kRR)}};

	for (auto& module : modules) {
		module.drive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		module.steer.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	}
	stopAllMotors(modules);

	if (!g_steerEncodersZeroed) {
		bool tare_failed = false;
		for (auto& module : modules) {
			if (module.steer.tare_position() == PROS_ERR) tare_failed = true;
		}
		if (tare_failed) {
			stopAllMotors(modules);
			autonStatus("ERROR: encoder tare failed", "Autonomous aborted");
			return;
		}
		g_steerEncodersZeroed = true;
	}

	pros::GPS gps(kGpsPort, kGpsXOffset, kGpsYOffset,
				  kGpsInitialX, kGpsInitialY, kGpsInitialHeadingDeg);
	gps.set_data_rate(kGpsDataRateMs);
	logGps(gps, "start");

	std::uint32_t loop_start_ms = pros::millis();

	// 1. Forward 1 m (robot frame: +y is forward).
	autonStatus("Step 1/4", "Forward 1 m");
	if (!driveDistance(modules, 0.0, 1.0, kAutonDistanceIn, loop_start_ms)) {
		stopAllMotors(modules);
		autonStatus("ERROR: forward leg", "Autonomous aborted");
		return;
	}

	// Bring the heading back to the starting heading (0) before the GPS settles.
	// Wheel drift during the forward leg can leave the robot slightly rotated,
	// and the GPS needs a steady, known heading to get a clear read.
	autonStatus("Step 1/4", "Correct heading to 0");
	if (!turnToGpsHeading(gps, modules, kGpsInitialHeadingDeg, loop_start_ms)) {
		stopAllMotors(modules);
		autonStatus("ERROR: heading correction", "Autonomous aborted");
		return;
	}

	// 2. Let the GPS settle while holding still.
	autonStatus("Step 2/4", "Stabilizing GPS");
	if (!stabilizeGps(gps, modules, loop_start_ms)) {
		stopAllMotors(modules);
		autonStatus("ERROR: GPS did not settle", "Autonomous aborted");
		return;
	}

	// 3. Rotate to a 90 degree heading.
	autonStatus("Step 3/4", "Turn to 90 deg");
	if (!turnToGpsHeading(gps, modules, kAutonTargetHeadingDeg, loop_start_ms)) {
		stopAllMotors(modules);
		autonStatus("ERROR: turn failed", "Autonomous aborted");
		return;
	}

	// 4. Strafe 1 m sideways (robot-frame x).
	autonStatus("Step 4/4", "Strafe 1 m");
	if (!driveDistance(modules, kFinalLegRobotVx, 0.0, kAutonDistanceIn, loop_start_ms)) {
		stopAllMotors(modules);
		autonStatus("ERROR: strafe leg", "Autonomous aborted");
		return;
	}

	stopAllMotors(modules);
	logGps(gps, "end");
	autonStatus("Routine complete", "");
}

/**
 * Runs the operator control code.
 */
void opcontrol() {
	
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	
	// Order here must match kModuleNames: FL, RL, FR, RR.
	std::array<SwerveModule, kNumModules> modules{{
		SwerveModule(cfg::kFL), SwerveModule(cfg::kRL),
		SwerveModule(cfg::kFR), SwerveModule(cfg::kRR)}};

	// CHANGED: brake modes are now consistent. Before, left_front_steer was set
	// to COAST while the other three steer motors were BRAKE (looks like a
	// leftover from testing), so one pod behaved differently from the rest.
	for (auto& module : modules) {
		module.drive.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
		module.steer.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	}
	// REMOVED: the extra set-everything-to-coast pass and the duplicate stop
	// commands that used to run before and after it.
	stopAllMotors(modules);

	// ------------------------------------------------------------ startup gate
	// Center the pods by hand, press X, encoders get zeroed. Returns when done.
	waitForPodAlignmentAndTare(master, modules);
	pros::screen::erase();
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "Mode: opcontrol");

	const double wheel_circumference_in = drivetrain::kPi * cfg::kWheelDiameterIn;
	int cycle_counter = 0;
	// CHANGED: fixed-period loop. pros::delay(20) at the end of the loop made
	// the period "work time + 20 ms", and the screen printing made it uneven.
	std::uint32_t loop_start_ms = pros::millis();

	while (true) {
		// ------------------------------------------------------- driver input
		const int left_y = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		const int left_x = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		const int right_x = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
		const double normalized_x = normalizedAxis(left_x);
		const double normalized_y = normalizedAxis(left_y);
		const double normalized_rotation = normalizedAxis(right_x);

		// Keep the translation vector inside the unit circle (diagonals would
		// otherwise be ~41% faster than straight lines).
		const double translation_magnitude = std::hypot(normalized_x, normalized_y);
		const double translation_scale =
			translation_magnitude > 1.0 ? 1.0 / translation_magnitude : 1.0;

		// CHANGED: kRotationSign flips the spin direction (stick right was turning
		// the robot left). It is applied to omega itself, so pure spin and
		// combined driving both flip together.
		const double clockwise_omega =
			normalized_rotation * cfg::kMaxAngularSpeedRadPerSec * cfg::kRotationSign;
		const drivetrain::ChassisCommand command{
			normalized_x * translation_scale * cfg::kMaxLinearSpeedInPerSec,
			normalized_y * translation_scale * cfg::kMaxLinearSpeedInPerSec,
			clockwise_omega};
		const bool chassis_stopped =
			normalized_x == 0.0 && normalized_y == 0.0 && normalized_rotation == 0.0;

		// -------------------------------------------- read each encoder ONCE
		std::array<ModuleCycle, kNumModules> cycle{};
		bool sensors_ok = true;
		for (std::size_t i = 0; i < kNumModules; ++i) {
			cycle[i].steerPositionDeg = modules[i].steer.get_position();
			// A disconnected motor reports +infinity; don't drive on garbage.
			if (!std::isfinite(cycle[i].steerPositionDeg)) sensors_ok = false;
		}
		if (!sensors_ok) {
			stopAllMotors(modules);
			pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "ERROR: steer sensor lost");
			pros::Task::delay_until(&loop_start_ms, cfg::kControlPeriodMs);
			continue;
		}

		// ------------------------------------------- kinematics + optimizing
		for (std::size_t i = 0; i < kNumModules; ++i) {
			auto& module = modules[i];
			auto& c = cycle[i];
			c.currentAngleRad = drivetrain::degToRad(
				c.steerPositionDeg / cfg::kSteerMotorRevPerModuleRev);

			// REMOVED: the "pure_rotation" block that negated every module
			// angle. It is now handled inside calculateModuleState().
			const drivetrain::ModuleState requested = calculateModuleState(
				command, module.config.position, chassis_stopped);
			c.state = optimizeModuleState(requested, c.steerPositionDeg,
										  c.currentAngleRad, chassis_stopped,
										  module.control, c.encoderTarget);

			// Low-pass the measured steer velocity for the coupling term below.
			const double measured_rpm = module.steer.get_actual_velocity();
			if (std::isfinite(measured_rpm)) {
				module.control.filteredSteerRpm +=
					cfg::kSteerVelocityFilterAlpha *
					(measured_rpm - module.control.filteredSteerRpm);
			}
		}

		// ------------------------------------- one global speed scale (in/s)
		// If any module is asked to go faster than the limit, slow ALL modules by
		// the same factor so the robot's direction of travel is preserved.
		double largest_module_speed = 0.0;
		for (const auto& c : cycle) {
			largest_module_speed = std::max(largest_module_speed, std::abs(c.state.speed));
		}
		const double speed_scale =
			largest_module_speed > cfg::kMaxLinearSpeedInPerSec
				? cfg::kMaxLinearSpeedInPerSec / largest_module_speed
				: 1.0;

		// ------------------------------------------------- drive motor RPMs
		// REMOVED: the second global scale (drive_speed_scale). It was computed
		// from the final RPMs, which include the measured steer velocity, so a
		// steering transient on ONE pod changed the speed of ALL four wheels,
		// and it also scaled the coupling correction itself. The speed limit is
		// now derived from the motor limit (see config), so the first scale
		// above is sufficient, and each motor is simply clamped at the end.
		for (std::size_t i = 0; i < kNumModules; ++i) {
			auto& module = modules[i];
			auto& c = cycle[i];

			// REMOVED: the align-then-drive delay (it added jitter). The wheel is back
			// to the original soft slow-down while the pod is still turning: full speed
			// when aligned, zero at 90 degrees off. To drop even that, set
			// alignment_scale to 1.0.
			const double steering_error = drivetrain::shortestAngularError(
				c.state.angleRad, c.currentAngleRad);
			const double alignment_scale = std::max(0.0, std::cos(steering_error));
			const double wheel_rpm = c.state.speed * alignment_scale * speed_scale /
									 wheel_circumference_in * 60.0 *
									 cfg::kDriveMotorRevPerWheelRev;

			// Turning the pod drags the wheel along; subtract that so the wheel
			// ground speed stays what we asked for. Uses the filtered, deadbanded
			// velocity and a tunable gain (see config).
			double steer_rpm = module.control.filteredSteerRpm;
			if (std::abs(steer_rpm) < cfg::kSteerVelocityDeadbandRpm) steer_rpm = 0.0;
			const double pod_rpm = steer_rpm / cfg::kSteerMotorRevPerModuleRev;
			const double steering_induced_drive_rpm =
				pod_rpm * cfg::kWheelRevPerPodRev * cfg::kDriveMotorRevPerWheelRev *
				cfg::kSteerCouplingCompensationGain;

			const double command_rpm = std::clamp(
				wheel_rpm - steering_induced_drive_rpm,
				-cfg::kDriveMotorMaxVelocityRpm, cfg::kDriveMotorMaxVelocityRpm);
			c.driveRpm = static_cast<int>(std::lround(command_rpm));
		}

		// ---------------------------------------------------- motor outputs
		for (std::size_t i = 0; i < kNumModules; ++i) {
			modules[i].drive.move_velocity(cycle[i].driveRpm);
			modules[i].steer.move_absolute(cycle[i].encoderTarget,
										   cfg::kSteerTargetVelocityRpm);
		}

		// Terminal printout of all four modules' motion + drive velocity commands.
		maybeLogModuleMotion(modules, cycle, "opcontrol");

		// ---------------------------------------------------- diagnostics
		if ((cycle_counter % (kScreenRefreshMs / static_cast<int>(cfg::kControlPeriodMs))) == 0) {
			char buffer[128];
			std::snprintf(buffer, sizeof(buffer), "Axes: LX=%d LY=%d RX=%d", left_x, left_y, right_x);
			pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "%s", buffer);

			if (normalized_x != 0.0 || normalized_y != 0.0) {
				std::snprintf(buffer, sizeof(buffer), "XY heading: %.0f deg",
							  drivetrain::compassDegrees(std::atan2(normalized_x, normalized_y)));
			} else {
				std::snprintf(buffer, sizeof(buffer), "XY heading: neutral");
			}
			pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 3, "%s", buffer);

			// CHANGED: the old test checked the same sign twice
			// (clockwise_omega > 0 and then normalized_rotation < 0).
			const char* rotation_direction =
				normalized_rotation > 0.0 ? "CW" : (normalized_rotation < 0.0 ? "CCW" : "OFF");
			std::snprintf(buffer, sizeof(buffer), "RX:%s FL%+d RL%+d FR%+d RR%+d",
						  rotation_direction, cycle[0].driveRpm, cycle[1].driveRpm,
						  cycle[2].driveRpm, cycle[3].driveRpm);
			pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 4, "%s", buffer);

			for (std::size_t i = 0; i < kNumModules; ++i) {
				std::snprintf(buffer, sizeof(buffer), "%s %.0f deg %+d rpm",
							  kModuleNames[i],
							  drivetrain::compassDegrees(cycle[i].state.angleRad),
							  cycle[i].driveRpm);
				pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5 + static_cast<int>(i), "%s", buffer);
			}

			std::snprintf(buffer, sizeof(buffer), "Ports FL%d/%d RL%d/%d FR%d/%d RR%d/%d",
						  cfg::kFL.drivePort, cfg::kFL.steerPort,
						  cfg::kRL.drivePort, cfg::kRL.steerPort,
						  cfg::kFR.drivePort, cfg::kFR.steerPort,
						  cfg::kRR.drivePort, cfg::kRR.steerPort);
			pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 9, "%s", buffer);

		}

		// Controller power display: every 3rd cycle (60 ms, above the controller's
		// ~50 ms minimum), alternating which line is updated. Never blocks.
		if (cycle_counter % 3 == 0) {
			updateControllerDisplay(master, modules, (cycle_counter / 3) % 2 == 0);
		}

		cycle_counter++;
		pros::Task::delay_until(&loop_start_ms, cfg::kControlPeriodMs);
	}
}