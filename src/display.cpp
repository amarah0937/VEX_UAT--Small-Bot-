#include "swerve/display.hpp"

#include <cstdio>
#include <cstring>

namespace swerve {

namespace {
constexpr std::uint32_t kControllerMinGapMs = 60;    // controller drops updates < ~50 ms apart
constexpr std::uint32_t kControllerRefreshMs = 1000; // resend unchanged lines occasionally
constexpr std::uint32_t kBrainRefreshMs = 200;
}  // namespace

// ------------------------------------------------------------ ControllerScreen
ControllerScreen::ControllerScreen(pros::Controller& controller) : controller_(controller) {}

void ControllerScreen::set(const int line, const char* text) {
	if (line < 0 || line >= kLines) return;
	Line& l = lines_[line];
	if (std::strncmp(l.text, text, kWidth) == 0 && l.text[0] != '\0') return;  // unchanged
	std::strncpy(l.text, text, kWidth);
	l.text[kWidth] = '\0';
	l.dirty = true;
}

void ControllerScreen::service() {
	const std::uint32_t now = pros::millis();
	if (now - lastAnyPrintMs_ < kControllerMinGapMs) return;

	for (int k = 0; k < kLines; ++k) {
		const int idx = (next_ + k) % kLines;
		Line& l = lines_[idx];
		if (l.text[0] == '\0') continue;
		if (l.dirty || now - l.lastPrintMs > kControllerRefreshMs) {
			controller_.print(idx, 0, "%-15s", l.text);  // padded so old characters are erased
			l.dirty = false;
			l.lastPrintMs = now;
			lastAnyPrintMs_ = now;
			next_ = (idx + 1) % kLines;
			return;  // one line per call
		}
	}
}

// ------------------------------------------------------------------ brain text
void brainPrint(const int line, const char* text) {
	pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, line, "%s", text);
}

// ----------------------------------------------------------------- DisplayTask
DisplayTask::DisplayTask(SwerveDrive& drive, pros::Controller& controller)
	: drive_(drive), screen_(controller) {}

void DisplayTask::start() {
	if (task_) return;
	// One below default priority so the control loop always wins.
	task_ = std::make_unique<pros::Task>([this]() { loop(); }, TASK_PRIORITY_DEFAULT - 1,
										 TASK_STACK_DEPTH_DEFAULT, "swerve_display");
}

void DisplayTask::loop() {
	std::uint32_t lastBrainMs = 0;
	char buf[96];

	while (true) {
		if (paused_) {
			pros::delay(20);
			continue;
		}
		const DriveTelemetry t = drive_.snapshot();

		// ---- controller: drive motor power (watts), front pair / rear pair / status
		// Module storage order is FL, RL, FR, RR.
		const double fl = drive_.module(0).drivePowerWatts();
		const double rl = drive_.module(1).drivePowerWatts();
		const double fr = drive_.module(2).drivePowerWatts();
		const double rr = drive_.module(3).drivePowerWatts();
		std::snprintf(buf, sizeof(buf), "FL%4.1f FR%4.1f", fl, fr);
		screen_.set(0, buf);
		std::snprintf(buf, sizeof(buf), "RL%4.1f RR%4.1f", rl, rr);
		screen_.set(1, buf);
		const char* status = !t.homed ? "NOT HOMED" : (!t.sensorsOk ? "SENSOR FAIL" : "OK");
		screen_.set(2, status);
		screen_.service();

		// ---- brain: diagnostics page, at 5 Hz
		const std::uint32_t now = pros::millis();
		if (now - lastBrainMs >= kBrainRefreshMs) {
			lastBrainMs = now;
			std::snprintf(buf, sizeof(buf), "SWERVE v2  %s", status);
			brainPrint(1, buf);
			std::snprintf(buf, sizeof(buf), "vx%+.2f vy%+.2f w%+.2f m/s,rad/s", t.command.vxMps,
						  t.command.vyMps, t.command.omegaRadPerSec);
			brainPrint(2, buf);
			std::snprintf(buf, sizeof(buf), "scale %.2f  dt %.1f ms", t.normalizationScale,
						  t.dtSec * 1000.0);
			brainPrint(3, buf);
			for (std::size_t i = 0; i < kNumModules; ++i) {
				const ModuleTelemetry& m = t.modules[i];
				std::snprintf(buf, sizeof(buf), "%s t%3.0f p%3.0f d%+4.0f c%+4.0f %c%c%c",
							  drive_.module(i).name(), m.targetDeg, m.podDeg, m.finalDriveRpm,
							  m.couplingDriveRpm, m.flipped ? 'F' : '-', m.held ? 'H' : '-',
							  m.saturated ? 'S' : '-');
				brainPrint(4 + static_cast<int>(i), buf);
			}
		}

		pros::delay(20);
	}
}

}  // namespace swerve
