#include "swerve/swerve_module.hpp"

#include <cmath>

namespace swerve {

namespace {

pros::v5::MotorGears gearsetFor(const Cartridge cartridge) {
	switch (cartridge) {
		case Cartridge::Red100:
			return pros::v5::MotorGears::red;
		case Cartridge::Blue600:
			return pros::v5::MotorGears::blue;
		case Cartridge::Green200:
		default:
			return pros::v5::MotorGears::green;
	}
}

// PROS reverses a motor when its port number is negative.
std::int8_t signedPort(const std::int8_t port, const bool reversed) {
	return static_cast<std::int8_t>(reversed ? -port : port);
}

}  // namespace

SwerveModule::SwerveModule(const ModuleConfig& config)
	: config_(config),
	  drive_(signedPort(config.drivePort, config.driveReversed),
			 gearsetFor(config::kDriveCartridge), pros::v5::MotorUnits::degrees),
	  steer_(signedPort(config.steerPort, config.steerReversed),
			 gearsetFor(config::kSteerCartridge), pros::v5::MotorUnits::degrees),
	  controller_(config::makeModuleControllerParams(config)) {}

void SwerveModule::configureMotors() {
	drive_.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
	steer_.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

bool SwerveModule::tareSteering() {
	// tare_position() reports failure as PROS_ERR, not a negative number.
	if (steer_.tare_position() == PROS_ERR) return false;
	resetController();
	return true;
}

void SwerveModule::resetController() { controller_.reset(); }

bool SwerveModule::sample() {
	const double positionDeg = steer_.get_position();
	const double velocityRpm = steer_.get_actual_velocity();

	// A disconnected motor reports +infinity; never drive on garbage.
	sensorValid_ = std::isfinite(positionDeg) && std::isfinite(velocityRpm);
	if (!sensorValid_) return false;

	// Motor coordinates -> canonical pod coordinates (the ONLY place the 2:1
	// steering reduction is applied to measurements).
	podContinuousRad_ = degToRad(positionDeg) / config::kSteerMotorRevPerPodRev;
	podRateRadPerSec_ = rpmToRadPerSec(velocityRpm) / config::kSteerMotorRevPerPodRev;
	return true;
}

void SwerveModule::compute(const ModuleRequest& request, const double dt) {
	output_ = controller_.step(request, podContinuousRad_, podRateRadPerSec_, dt);
	telemetry_ = output_.telemetry;
	telemetry_.sensorValid = sensorValid_;
}

void SwerveModule::commit() {
	drive_.move_velocity(static_cast<int>(std::lround(output_.driveMotorRpm)));
	steer_.move_velocity(static_cast<int>(std::lround(output_.steerMotorRpm)));
}

void SwerveModule::stop() {
	drive_.move_velocity(0);
	steer_.move_velocity(0);
	output_ = ModuleOutput{};
}

double SwerveModule::drivePowerWatts() { return drive_.get_power(); }

}  // namespace swerve
