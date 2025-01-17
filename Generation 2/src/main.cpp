#include "main.h"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/misc.h"
#include "lemlib/api.hpp"
#include "pros/screen.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({-5, -8, -10}, pros::MotorGearset::blue); // motors use 600 RPM cartridges
pros::MotorGroup right_motors({14, 3, 18}, pros::MotorGearset::blue); 

pros::v5::Rotation armRotation(1);

pros::Motor bottom(-2, pros::v5::MotorGears::green);
pros::Motor top(20, pros::v5::MotorGears::green);
pros::v5::Optical optical(9);

pros::Motor arm(19, pros::v5::MotorGears::green);
pros::adi::DigitalOut doinker('C');
pros::adi::DigitalOut intakeRaise('B');
pros::adi::DigitalOut clamp('A');

pros::Rotation horizontalEncoder(13);
pros::Rotation verticalEncoder(12);

pros::IMU imu(9);

lemlib::TrackingWheel horizontalTrackingWheel(
	&horizontalEncoder, 
	lemlib::Omniwheel::NEW_2, 
	0 // change distance
);
lemlib::TrackingWheel verticalTrackingWheel(
	&verticalEncoder, 
	lemlib::Omniwheel::NEW_2, 
	0 // change distance
);

lemlib::Drivetrain drivetrain(
	&left_motors, // left motor group
	&right_motors, // right motor group
	11.3, // 10 inch track width
	lemlib::Omniwheel::NEW_275, // using new 2.75" omnis
	450, // drivetrain rpm is 450
	2 // horizontal drift is 2 (for now)
);

lemlib::OdomSensors sensors(&verticalTrackingWheel, 
	nullptr, 
	&horizontalTrackingWheel, 
	nullptr, 
	&imu
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller, sensors);


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
void autonomous() {}

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
	pros::screen::print(pros::E_TEXT_MEDIUM, 3, "X: %d", sensors.horizontal1->getDistanceTraveled());
}