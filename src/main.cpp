#include "main.h"

#include <algorithm>
#include <cstdint>

#include "swerve/config.hpp"
#include "swerve/display.hpp"
#include "swerve/startup.hpp"
#include "swerve/swerve_drive.hpp"

namespace {

// Measured loop period in seconds, clamped so one slow cycle cannot make the
// slew/profile maths jump (Doc 1 section 6.1).
double measureDt(std::uint32_t& lastMs) {
	const std::uint32_t now = pros::millis();
	double dt = static_cast<double>(now - lastMs) / 1000.0;
	lastMs = now;
	if (dt <= 0.0) dt = swerve::config::kControlPeriodMs / 1000.0;
	return std::clamp(dt, swerve::config::kDtMinSec, swerve::config::kDtMaxSec);
}

}  // namespace

void initialize() {
	pros::screen::set_pen(pros::Color::white);
	pros::screen::erase();
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "Swerve v2");
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "Initializing");
}

void disabled() {
	pros::screen::erase();
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "Mode: disabled");
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 2, "Drivetrain outputs off");
}

void competition_initialize() {
	pros::screen::erase();
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "Mode: competition init");
}

// Calibration autonomous: pods must be centered at program start (the same
// pre-match procedure as the X-press). Drives straight forward for one meter's
// worth of time at a fixed ground speed. Measure what it really covered: if it
// is not ~1 m the wheel diameter or drive ratio in config.hpp is off.
void autonomous() {
	pros::screen::erase();
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 1, "Mode: autonomous (1 m test)");

	swerve::SwerveDrive drive;
	drive.configure();
	if (!drive.homeSteering()) {
		drive.stop();
		return;
	}

	constexpr double kDistanceM = 1.0;
	const double speedMps = std::min(0.25, 0.7 * swerve::config::kMaxWheelSpeedMps);
	const std::uint32_t durationMs = static_cast<std::uint32_t>(kDistanceM / speedMps * 1000.0);

	const swerve::ChassisCommand forward{0.0, speedMps, 0.0};
	std::uint32_t loopStartMs = pros::millis();
	std::uint32_t lastMs = loopStartMs;
	const std::uint32_t startMs = loopStartMs;
	while (pros::millis() - startMs < durationMs) {
		drive.driveChassis(forward, measureDt(lastMs));
		pros::Task::delay_until(&loopStartMs, swerve::config::kControlPeriodMs);
	}
	drive.stop();
}

void opcontrol() {
	// These are static on purpose: PROS deletes the opcontrol task when the
	// match mode changes WITHOUT running destructors, and the display task keeps
	// reading drive/master. Static storage can never dangle.
	static pros::Controller master(pros::E_CONTROLLER_MASTER);
	static swerve::SwerveDrive drive;
	static swerve::DisplayTask display(drive, master);

	drive.configure();

	// Center pods by hand, press X, encoders zero. Drive is inhibited until then.
	// The display task is paused so the startup prompt owns the screens.
	display.setPaused(true);
	if (!swerve::alignAndHome(master, drive)) return;
	pros::screen::erase();
	display.setPaused(false);

	// Diagnostics run in their own low-priority task so they can never stall the loop.
	display.start();

	std::uint32_t loopStartMs = pros::millis();
	std::uint32_t lastMs = loopStartMs;

	while (true) {
		swerve::RawInput raw;
		raw.forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		raw.strafe = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
		raw.rotate = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		drive.periodic(raw, measureDt(lastMs));

		pros::Task::delay_until(&loopStartMs, swerve::config::kControlPeriodMs);
	}
}
