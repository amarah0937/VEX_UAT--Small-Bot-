#include "swerve/startup.hpp"

#include "swerve/display.hpp"

namespace swerve {

namespace {

void drawBrain(const char* l1, const char* l2, const char* l3, const bool clearFirst) {
	if (clearFirst) pros::screen::erase();
	brainPrint(1, l1);
	brainPrint(2, l2);
	brainPrint(3, l3);
}

}  // namespace

bool alignAndHome(pros::Controller& controller, SwerveDrive& drive) {
	pros::screen::set_pen(pros::Color::white);
	pros::screen::set_eraser(pros::Color::black);

	ControllerScreen screen(controller);
	const char* p1 = "Mode: opcontrol";
	const char* p2 = "Center all four pods";
	const char* p3 = "Press X to zero encoders";

	drawBrain(p1, p2, p3, true);
	screen.set(0, "Center pods");
	screen.set(1, "Press X to zero");

	// If X is still held from before, wait for release so we cannot exit at once.
	while (controller.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
		screen.service();
		pros::delay(20);
	}

	// Wait for a fresh press. The prompt is re-sent regularly (brain every 100 ms,
	// controller by ControllerScreen) so it can never end up blank.
	int pass = 0;
	while (!controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
		screen.service();
		if (++pass % 5 == 0) drawBrain(p1, p2, p3, false);
		pros::delay(20);
	}

	if (!drive.homeSteering()) {
		drive.stop();
		screen.set(0, "TARE FAILED");
		screen.set(1, "Restart robot");
		while (true) {
			drawBrain("ERROR: encoder tare failed", "Outputs remain stopped",
					  "Restart and check motors", false);
			screen.service();
			pros::delay(20);
		}
	}

	// Hold the confirmation for 1.5 s, still servicing the screens.
	screen.set(0, "Encoders zeroed");
	screen.set(1, "Control active");
	drawBrain(p1, "Encoder zero established", "Four-pod control active", true);
	for (int i = 0; i < 75; ++i) {  // 75 x 20 ms
		screen.service();
		pros::delay(20);
	}
	return true;
}

}  // namespace swerve
