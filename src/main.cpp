#include "main.h"
#include <iostream>
/** 
 * My general idea
 * I already have a pidf controller and a localizer set up.
 * I want to use piecewise parametric equations to define a path
 * define the robots closest point on the path,
 * add a constant vaue to the t value of that point,
 * then point the robots heading at the point that the t + x value gives me
 * define a maximum t value which the robot can jump for smoothing curves
 * define the robots translational movement as inverse to the heading error with a minimum value
 * To see the curve and tools for makeing your own see https://www.desmos.com/calculator/rjjdispdfb 
 * or see https://visualizer.pedropathing.com for a better view
 */

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {

	printf("%c", 0xAA); // start program on jet brains
	fflush(stdout);

	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
	PIDFCoefficients coeff = PIDFCoefficients();
	coeff.P = 0;
	coeff.I = 0;
	coeff.D = 0;
	coeff.F = 0;
	Follower follower(DriveLocalizerConstants(), coeff, Pose());
	while (true) {
		return;
	}
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
	DriveLocalizerConstants constants = DriveLocalizerConstants();
	PIDFCoefficients coeff = PIDFCoefficients();

	coeff.P = 0.47;
	coeff.I = 0.01;
	coeff.D = 0.022;
	coeff.F = 0.09;

	Follower follower = Follower(constants, coeff, Pose(-63.5, -48, 0));
	// Follower follower = Follower(constants, coeff, Pose(48, -48, 0));

	pros::Controller master(pros::E_CONTROLLER_MASTER);

	// follower.followCurveChain(CurveChain({Paths::toRightWallZig, Paths::toBottomWallZigy, Paths::toLeftWallZag}));
	// follower.followCurve(Paths::toRightWallZig);
	
	std::uint32_t lastPrintTime = -500;

	Pose lastCamPose = Pose();

	while (true) {

		follower.update();
		if (pros::millis() - lastPrintTime >= 1000) {
			Pose robotPose = follower.getPose();
            std::cout << "Robot's Pose: " << robotPose << "\n";\
			if (follower.aTagManager->getRobotPose(follower.getPose()).distFrom(follower.getPose()) > 0.0001) {
				lastCamPose = follower.aTagManager->getRobotPose(follower.getPose());
			}
            std::cout << "Camera's Pose: " << lastCamPose << "\n";
            lastPrintTime = pros::millis();
        }

		double forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / 127.0;
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / 127.0 / 2.0;

		follower.drivetrain->setLeftPowerSlew((forward + turn));
		follower.drivetrain->setRightPowerSlew((forward - turn));

		pros::delay(40);

	}
}
		/// stored code 9/9
		// pros::lcd::print(1, "P: %0.6f.   I: %0.6f", pidf.P(), pidf.I());
		// pros::lcd::print(2, "D: %0.6f.   F: %0.6f", pidf.D(), pidf.F());
		// pros::lcd::print(3, "Pose: (%0.2f, %0.2f) H: %0.2f", localizer.getPose().x, localizer.getPose().y, (localizer.getPose().heading * 180/std::numbers::pi));
		// pros::lcd::print(4, "Power: %0.6f", power);
		// pros::lcd::print(5, "Error: %0.6f", pidf.getError());

		// if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
		// 	pidf.setP(pidf.P() + 0.01);
		// }
		// if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
		// 	pidf.setD(pidf.D() + 0.0005);
		// }
		// if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
		// 	pidf.setP(pidf.P() - 0.01);
		// }
		// if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
		// 	pidf.setD(pidf.D() - 0.0005);
		// }
		// if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
		// 	pidf.setI(pidf.I() + 0.0005);
		// }
		// if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
		// 	pidf.setI(pidf.I() - 0.0005);
		// }


/*
	Their code
	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::MotorGroup left_mg({1, -2, 3});    // Creates a motor group with forwards ports 1 & 3 and reversed port 2
	pros::MotorGroup right_mg({-4, 5, -6});  // Creates a motor group with forwards port 5 and reversed ports 4 & 6


	while (true) {
		pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
		                 (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
		                 (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);  // Prints status of the emulated screen LCDs

		// Arcade control scheme
		int dir = master.get_analog(ANALOG_LEFT_Y);    // Gets amount forward/backward from left joystick
		int turn = master.get_analog(ANALOG_RIGHT_X);  // Gets the turn left/right from right joystick
		left_mg.move(dir - turn);                      // Sets left motor voltage
		right_mg.move(dir + turn);                     // Sets right motor voltage
		pros::delay(20);                               // Run for 20 ms then update
	}
	*/