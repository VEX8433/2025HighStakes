#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "Subsystems/intake.hpp"

pros::Motor LEFT_MIDDLE(-8, pros::v5::MotorGears::blue);
pros::Motor LEFT_FRONT(-5, pros::v5::MotorGears::blue);
pros::Motor LEFT_BACK(-10, pros::v5::MotorGears::blue);
pros::Motor RIGHT_MIDDLE(3, pros::v5::MotorGears::blue);
pros::Motor RIGHT_FRONT(14, pros::v5::MotorGears::blue);
pros::Motor RIGHT_BACK(18, pros::v5::MotorGears::blue);

pros::Motor bottom(-2, pros::v5::MotorGears::green);
pros::Motor top(20, pros::v5::MotorGears::green);
pros::v5::Optical optical(9);

pros::Motor arm(19, pros::v5::MotorGears::green);
pros::adi::DigitalOut doinker('C');
pros::adi::DigitalOut intakeRaise('B');
pros::adi::DigitalOut clamp('A');

pros::v5::Rotation armRotation(1);

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
	pros::lcd::initialize(); // initialize brain screen
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

pros::Controller master(pros::E_CONTROLLER_MASTER);

void intaketelop(){
	bool thing = false;
    while(true){
		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			bottom.move_velocity(-200);
			top.move_velocity(-200);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			bottom.move_velocity(200);
			top.move_velocity(200);
			
			// if(optical.get_hue() < 240 && optical.get_hue() > 200){
			// 	pros::delay(70);
			// 	top.move_velocity(0);
			// 	pros::delay(150);
			// 	top.move_velocity(600);
			// }

			pros::delay(150);

			if(top.get_actual_velocity() < 20){
				top.move_velocity(-200);
				pros::delay(150);
				top.move_velocity(200);
			}
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			top.move_velocity(0);
			bottom.move_velocity(200);
		} 
		else{
			top.move_velocity(0);
			bottom.move_velocity(0);
		}
	}
	pros::delay(20);
}

void opcontrol() {
	optical.set_led_pwm(100);

	arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


	armRotation.reset_position();

	LEFT_MIDDLE.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	LEFT_FRONT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	LEFT_BACK.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	RIGHT_MIDDLE.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	RIGHT_FRONT.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
	RIGHT_BACK.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

	bool intakeToggle = false;
	bool clampToggle = false;
	bool doinkerToggle = false;
	bool armToggle = false;
	bool intakeTopToggle = false;

	pros::Task intakeTask(intaketelop);

	while (true) {

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)){
			armToggle = true;
		}

		if(armToggle && armRotation.get_position() < 4500){
			arm.move_velocity(100);
		}
		else if(armToggle && armRotation.get_position() >= 4500){
			armToggle = false;
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)){
			clampToggle = !clampToggle;
			clamp.set_value(clampToggle);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)){
			doinkerToggle = !doinkerToggle;
			doinker.set_value(doinkerToggle);
		}

		if(master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)){
			intakeToggle = !intakeToggle;
			intakeRaise.set_value(intakeToggle);
		}
	
		// intake.telOP(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1), master.get_digital(pros::E_CONTROLLER_DIGITAL_R2));

		if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			arm.move_velocity(200);
		}
		else if(master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			arm.move_velocity(-200);
		}
		else if(!armToggle){
			arm.move_velocity(0);
		}

		// LeftMotor.spin((Controller1.Axis3.value() + Controller1.Axis1.value()*2));
		// RightMotor.spin((Controller1.Axis3.value() - Controller1.Axis1.value()*2));

		int LeftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
		int RightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

		LEFT_MIDDLE.move_velocity((LeftY + RightX) * 4.7244);
		LEFT_FRONT.move_velocity((LeftY + RightX) * 4.7244);
		LEFT_BACK.move_velocity((LeftY + RightX) * 4.7244);
		RIGHT_MIDDLE.move_velocity((LeftY - RightX) * 4.7244);
		RIGHT_BACK.move_velocity((LeftY - RightX) * 4.7244);
		RIGHT_FRONT.move_velocity((LeftY - RightX) * 4.7244);

		pros::lcd::print(0, "%d", armRotation.get_position());
		pros::delay(20);
	}
}