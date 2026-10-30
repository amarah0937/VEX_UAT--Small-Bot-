#include "swerve/swerve_drive.hpp"

namespace swerve {

SwerveDrive::SwerveDrive()
	: modules_{{SwerveModule(config::kModules[0]), SwerveModule(config::kModules[1]),
				SwerveModule(config::kModules[2]), SwerveModule(config::kModules[3])}},
	  positions_{{config::kModules[0].position, config::kModules[1].position,
				  config::kModules[2].position, config::kModules[3].position}},
	  conditioner_(config::makeInputParams()) {}

void SwerveDrive::configure() {
	for (auto& m : modules_) m.configureMotors();
	stop();
}

bool SwerveDrive::homeSteering() {
	bool ok = true;
	for (auto& m : modules_) {
		if (!m.tareSteering()) ok = false;
	}
	homed_ = ok;
	conditioner_.reset();
	telemetry_.homed = homed_;
	return ok;
}

void SwerveDrive::stop() {
	for (auto& m : modules_) m.stop();
}

bool SwerveDrive::periodic(const RawInput& raw, const double dt) {
	const ChassisCommand command = conditioner_.update(raw, dt);
	return runCycle(command, dt);
}

bool SwerveDrive::driveChassis(const ChassisCommand& command, const double dt) {
	return runCycle(command, dt);
}

bool SwerveDrive::runCycle(const ChassisCommand& command, const double dt) {
	telemetry_.homed = homed_;
	telemetry_.command = command;
	telemetry_.dtSec = dt;

	// Drive is inhibited until the steering encoders have a known zero.
	if (!homed_) {
		stop();
		return false;
	}

	// ---- read every steering sensor exactly once
	bool sensorsOk = true;
	for (auto& m : modules_) {
		if (!m.sample()) sensorsOk = false;
	}
	telemetry_.sensorsOk = sensorsOk;
	if (!sensorsOk) {
		stop();  // fail safe: nothing drives or compensates on bad feedback
		return false;
	}

	// ---- kinematics + ONE global speed normalization
	const KinematicsResult kin = inverseKinematics(command, positions_, config::kMaxWheelSpeedMps);
	telemetry_.normalizationScale = kin.scale;

	// ---- compute all four, THEN commit all four
	for (std::size_t i = 0; i < kNumModules; ++i) {
		modules_[i].compute(kin.modules[i], dt);
	}
	for (std::size_t i = 0; i < kNumModules; ++i) {
		modules_[i].commit();
		telemetry_.modules[i] = modules_[i].telemetry();
	}
	return true;
}

}  // namespace swerve
