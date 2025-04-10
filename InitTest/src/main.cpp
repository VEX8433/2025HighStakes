#include "main.h"
#include <cmath>
#include "pros/misc.h"
#include "pros/adi.hpp"
#include "pros/imu.hpp"
#include "pros/motors.h"
#include "pros/rotation.hpp"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "pros/screen.h"
#include "lemlib/api.hpp"


// Global objects
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup left_motors({-5, -8, -10}, pros::MotorGearset::blue);
pros::MotorGroup right_motors({14, 3, 18}, pros::MotorGearset::blue);
pros::v5::Rotation armRotation(1);
pros::Motor bottom(-2, pros::v5::MotorGears::green);
pros::Motor top(20, pros::v5::MotorGears::green);
pros::v5::Optical optical(9);
pros::Motor arm(19, pros::v5::MotorGears::green);
pros::adi::DigitalOut doinker('C');
pros::adi::DigitalOut intakeRaise('B');
pros::adi::DigitalOut clamp('A');
pros::adi::DigitalOut ejector('D');

pros::Rotation horizontalEncoder(13);
pros::Rotation verticalEncoder(12);
pros::IMU imu(7);


lemlib::Drivetrain drivetrain(
	&left_motors, // left motor group
	&right_motors, // right motor group
	10.5, // 10 inch track width
	lemlib::Omniwheel::NEW_325, // using new 4" omnis
	360, // drivetrain rpm is 360
	2 // horizontal drift is 2 (for now)
);

lemlib::TrackingWheel horizontal_tracking_wheel(&horizontalEncoder, lemlib::Omniwheel::NEW_2, 3);
lemlib::TrackingWheel vertical_tracking_wheel(&verticalEncoder, lemlib::Omniwheel::NEW_2, 0);

lemlib::OdomSensors sensors(
	&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
	nullptr, 
    &horizontal_tracking_wheel,
    nullptr,
    // vertical tracking wheel 2, set to nullptr as we are using IMEs
	&imu // inertial sensor
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

lemlib::Chassis chassis(drivetrain, // drivetrain settings
					lateral_controller, // lateral PID settings
					angular_controller, // angular PID settings
					sensors // odometry sensors
);

void printBrain(){
    while(true){
        pros::screen::print(pros::E_TEXT_MEDIUM, 0, "X: %.2f", chassis.getPose().x);
        pros::screen::print(pros::E_TEXT_MEDIUM, 1, "Y: %.2f", chassis.getPose().y);
        pros::screen::print(pros::E_TEXT_MEDIUM, 2, "Theta: %.2f", chassis.getPose().theta);
        pros::screen::print(pros::E_TEXT_MEDIUM, 3, "%i", armRotation.get_position());
        pros::screen::print(pros::E_TEXT_MEDIUM, 4, "VERT VAL: %i", verticalEncoder.get_position());
        pros::screen::print(pros::E_TEXT_MEDIUM, 5, "HORI VAL: %i", horizontalEncoder.get_position());
        pros::screen::print(pros::E_TEXT_MEDIUM, 6, "Heading: %.2f", imu.get_heading());
        pros::screen::print(pros::E_TEXT_MEDIUM, 7, "HUE: %.2f", optical.get_hue());
        pros::screen::print(pros::E_TEXT_MEDIUM, 8, "Distance: %i", optical.get_proximity());
    }
}

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double kp = 0.006;
double ki = 0;
double kd = 0.00001;
double start_i = 30;

double output = 0;
double cur;
double error = 0;
double target = 6700;
double prev_error = 0;
double integral = 0;
double derivative = 0;

bool achieved = false;
bool ring = false;

double armPID(double current){
	error = target - current;
    derivative = error - prev_error;

    if(achieved == false && std::abs(error) < 500){
        achieved = true;
    }

    if(achieved && std::abs(error) > 1000){ // pick up ring threshold
        ring = true;
    }

    if(ring){
        return 0;
    }
    
    if (ki != 0) {
        if (fabs(error) < start_i){
            integral += error;
        }
        if (sgn(error) != sgn(prev_error)){
            integral = 0;
        }
    }

    output = (error * kp) + (integral * ki) + (derivative * kd);

    prev_error = error;

  return output;
}

double armPIDAuto(double current, int newTarget){
	if(newTarget == 0){
        error = target - current;
    }
    else{
        error = newTarget - current;
    }
    derivative = error - prev_error;

    if(achieved == false && std::abs(error) < 500){
        achieved = true;
    }

    if(achieved && std::abs(error) > 2000){ // pick up ring threshold
        ring = true;
    }

    if(ring){
        return 0;
    }
    
    if (ki != 0) {
        if (fabs(error) < start_i){
            integral += error;
        }
        if (sgn(error) != sgn(prev_error)){
            integral = 0;
        }
    }

    output = (error * kp) + (integral * ki) + (derivative * kd);

    prev_error = error;

  return output;
}

bool spinTop = false;
bool spinBottom = false;
bool reverse = false;
void intakeAuton(){
    while (true) {
        if(reverse){
            bottom.move_velocity(0);
            top.move_velocity(-200);
        }
        else if (spinTop && spinBottom) {
            bottom.move_velocity(200);
            top.move_velocity(200);

            pros::delay(100);

            if (top.get_actual_velocity() < 20) {
                top.move_velocity(-200);
                pros::delay(120);
                top.move_velocity(200);
            }
        } else if (spinBottom && !spinTop) {
            top.move_velocity(0);
            bottom.move_velocity(200);
        }else if (!spinBottom && spinTop){
            top.move_velocity(200);
            bottom.move_velocity(0);
        } else {
            top.move_velocity(0);
            bottom.move_velocity(0);
        }
        pros::delay(20);
    }
}

bool prime = false;
bool holdHigh = false;
void armAuton(int Pos){
    while(true){
        if(prime){
            double thing = armPIDAuto(armRotation.get_position(), 0);
            if(thing != 0){
                arm.move_velocity(thing);
            }
            else{
                arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                prime = false;
                arm.move_velocity(0);
            }
        }
        else if(holdHigh){
            double thing = armPIDAuto(armRotation.get_position(), 20000);
            if(thing != 0){
                arm.move_velocity(thing);
            }
            else{
                arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                prime = false;
                arm.move_velocity(0);
            }
        }


        pros::delay(20);
    }
}

void intakeTelop() {
    while (true) {
        // if(optical.get_hue() > 0 && optical.get_hue() < 10 && optical.get_proximity() == 255){ // red
        //     top.move_velocity(200);
        //     ejector.set_value(true);
        //     pros::delay(500);
        //     ejector.set_value(false);
        // }
        if(optical.get_hue() > 220 && optical.get_hue() < 230 && optical.get_proximity() == 255){// blue
            top.move_velocity(200);
            ejector.set_value(true);
            pros::delay(500);
            ejector.set_value(false);
        }
        if(master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            bottom.move_velocity(-200);
            top.move_velocity(-200);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            bottom.move_velocity(200);
            top.move_velocity(200);

            pros::delay(150);
            
            if (top.get_actual_velocity() < 20) {
                top.move_velocity(-200);
                pros::delay(120);
                top.move_velocity(200);
            }
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            top.move_velocity(0);
            bottom.move_velocity(200);
        } else {
            top.move_velocity(0);
            bottom.move_velocity(0);
        }
        pros::delay(5);
    }
}

bool armToggle = false;
void armTelop(){
    while(true){
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            armToggle = true;
            achieved = false;
            ring = false;
        }

        if(armToggle){
            double thing = armPID(armRotation.get_position());
            if(thing != 0){
                arm.move_velocity(thing);
            }
            else{
                arm.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
                arm.move_velocity(0);
            }
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            arm.move_velocity(200);
            armToggle = false;
            achieved = false;
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            arm.move_velocity(-200);
            armToggle = false;
            achieved = false;
        }else if (!armToggle) {
            arm.move_velocity(0);
            arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
            achieved = false;
        }
    }
}

//__________________________________________________________________________________________________________________
// pros::Task intakeAutonControl(intakeAuton);
// pros::Task armAutonControl(armAuton);
// pros::Task armTelopControl(armTelop);
// pros::Task intakeTelopControl(intakeTelop);

void initialize(){
    // pros::Task updateScreen(printBrain);
    imu.reset();
    chassis.calibrate();
    armRotation.reset_position();
    optical.set_led_pwm(10);
    chassis.setPose(0, 0, 0);
}

void opcontrol() {
    // intakeAutonControl.suspend();
    // armAutonControl.suspend();
    // armTelopControl.resume();
    // intakeTelopControl.resume();
    pros::Task armTelopControl(armTelop);
    pros::Task intakeTelopControl(intakeTelop);
    // armRotation.set_position(8500);


    arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);

    // left_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    // right_motors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);

    bool intakeToggle = false;
    bool clampToggle = false;
    bool doinkerToggle = false;
    bool intakeTopToggle = false;

    while (true) {

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            chassis.setPose(0, 0, 0);
            chassis.moveToPose(50, 50, 90, 3000);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            clampToggle = !clampToggle;
            clamp.set_value(clampToggle);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            doinkerToggle = !doinkerToggle;
            doinker.set_value(doinkerToggle);
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
            intakeToggle = !intakeToggle;
            intakeRaise.set_value(intakeToggle);
        }

        int LeftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int RightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        left_motors.move_velocity((LeftY + RightX) * 4.7244);
        right_motors.move_velocity((LeftY - RightX) * 4.7244);

        pros::delay(20);
    }
}


//____________________________________________________________________________________________________________________

void skills(){
    chassis.setPose(0, 0, 180);

    arm.move_velocity(200);
    pros::delay(800);
    arm.move_velocity(0);
    
    chassis.moveToPose(0, 6, 180, 700, {.forwards = false, .minSpeed = 30}, false);
    
    arm.move_velocity(-200);
    
    chassis.turnToHeading(90, 700);
    chassis.moveToPoint(-10, 6,1000, {.forwards = false, .maxSpeed = 100}, false);
    arm.move_velocity(0);

    chassis.moveToPoint(-20, 6, 700, {.forwards = false, .maxSpeed = 40}, false);
    chassis.moveToPoint(-23, 6, 300, {.forwards = false});
    clamp.set_value(true);
    pros::delay(200);

    chassis.turnToHeading(0, 1000);
    spinTop = true;
    spinBottom = true;
    chassis.moveToPose(-20, 35, 0, 1000, {.maxSpeed = 70, .minSpeed = 30});
    chassis.turnToHeading(-45, 1000, {.minSpeed = 30}, false);
    chassis.moveToPose(-40, 80, 0, 2000, {.minSpeed = 60}, false);
    prime = true;

    chassis.moveToPoint(-40, 50, 0, {.forwards = false}, false);
    holdHigh = true;

}

void goalSideBlue(){
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(-40, 600);

    arm.move_velocity(200);
    pros::delay(800);
    arm.move_velocity(0);

    chassis.moveToPoint(0, -15, 1000, {.forwards = false}, false);
    arm.move_velocity(-200);
    chassis.turnToHeading(-20, 600);
    chassis.moveToPoint(2, -32, 1000, {.forwards = false, .maxSpeed = 50}, false);
    arm.move_velocity(0);
    clamp.set_value(true);
    pros::delay(300);

    chassis.turnToHeading(90, 700);
    spinTop = true;
    spinBottom = true;
    chassis.moveToPoint(23, -32, 2000, {.maxSpeed = 70});
    pros::delay(500);
    chassis.turnToHeading(-50, 1000);

    intakeRaise.set_value(true);
    chassis.moveToPoint(-5, -20, 2000, {.maxSpeed = 70}, false);
    chassis.moveToPoint(-22, -8, 1000, {.maxSpeed = 30}, false);
    intakeRaise.set_value(false);

    pros::delay(300);
    chassis.moveToPoint(-10, -15, 1000, {.forwards = false});
    chassis.turnToHeading(-150, 1000);
    chassis.moveToPoint(-20, -30, 1000, {.maxSpeed = 60});
}

void ringSideBlue(){
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(40, 600);

    arm.move_velocity(200);
    pros::delay(800);
    arm.move_velocity(0);

    chassis.moveToPoint(0, -15, 1000, {.forwards = false}, false);
    arm.move_velocity(-200);
    chassis.turnToHeading(20, 600);
    chassis.moveToPoint(-4, -32, 1000, {.forwards = false, .maxSpeed = 50}, false);
    arm.move_velocity(0);
    clamp.set_value(true);
    pros::delay(300);

    chassis.turnToHeading(-90, 700);
    spinTop = true;
    spinBottom = true;
    chassis.moveToPoint(-20, -30, 2000, {.maxSpeed = 70});

    chassis.turnToHeading(180, 700);
    chassis.moveToPoint(-22, -39, 1000, {.maxSpeed = 60});
    chassis.moveToPoint(-22, -30, 2000, {.forwards = false, .maxSpeed = 70});

    chassis.turnToHeading(-30, 1000);
    chassis.moveToPoint(-30, -2, 500, {.minSpeed = 127});
    chassis.moveToPoint(-40, 8, 1000, {.maxSpeed = 50}, false);
    pros::delay(200);
    chassis.moveToPoint(-10, -25, 2000, {.forwards = false, .minSpeed = 70});
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(11, -25, 2000, {.maxSpeed = 50});
}

void goalSideRed(){
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(40, 600);

    arm.move_velocity(200);
    pros::delay(800);
    arm.move_velocity(0);

    chassis.moveToPoint(0, -15, 1000, {.forwards = false}, false);
    arm.move_velocity(-200);
    chassis.turnToHeading(20, 600);
    chassis.moveToPoint(-5, -32, 1000, {.forwards = false, .maxSpeed = 50}, false);
    arm.move_velocity(0);
    clamp.set_value(true);
    pros::delay(300);

    chassis.turnToHeading(-90, 700);
    spinTop = true;
    spinBottom = true;
    chassis.moveToPoint(-20, -30, 2000, {.maxSpeed = 70});
    chassis.turnToHeading(-50, 1000);
    intakeRaise.set_value(true);
    chassis.moveToPoint(10, -15, 2000, {.maxSpeed = 70}, false);
    chassis.moveToPoint(22, -12, 1000, {.maxSpeed = 30}, false);
    intakeRaise.set_value(false);
    pros::delay(500);
    chassis.moveToPoint(10, -15, 1000, {.forwards = false});
    chassis.moveToPoint(-5, -35, 2000, {.forwards = false, .maxSpeed = 70});
    chassis.turnToHeading(90, 1000);
    chassis.moveToPoint(20, -35, 2000, {.maxSpeed = 40});
}

void ringSideRed(){
    chassis.setPose(0, 0, 0);
    chassis.turnToHeading(-40, 600);

    arm.move_velocity(200);
    pros::delay(800);
    arm.move_velocity(0);

    chassis.moveToPoint(0, -15, 1000, {.forwards = false}, false);
    arm.move_velocity(-200);
    chassis.turnToHeading(-20, 600);
    chassis.moveToPoint(3, -32, 1000, {.forwards = false, .maxSpeed = 50}, false);
    arm.move_velocity(0);
    clamp.set_value(true);
    pros::delay(300);

    chassis.turnToHeading(90, 700);
    spinTop = true;
    spinBottom = true;
    chassis.moveToPoint(24, -32, 2000, {.maxSpeed = 80});
    pros::delay(500);

    chassis.turnToHeading(180, 700);
    chassis.moveToPoint(26, -44, 1000, {.maxSpeed = 60});
    chassis.moveToPoint(26, -30, 2000, {.forwards = false, .maxSpeed = 70});

    chassis.turnToHeading(30, 1000);
    chassis.moveToPoint(40, -2, 700, {.minSpeed = 127});
    chassis.moveToPoint(50, 8, 1000, {.maxSpeed = 50}, false);
    pros::delay(500);
    chassis.moveToPoint(10, -33, 2000, {.forwards = false, .maxSpeed = 80});
    chassis.turnToHeading(-93, 1000);
    chassis.moveToPoint(-12, -33, 2000, {.maxSpeed = 60});
}

void goalRushBlue(){
    chassis.setPose(0, 0, 23);
    spinBottom = true;
    chassis.moveToPoint(13, 30, 2000, {.minSpeed = 127}, false);
    doinker.set_value(true);
    chassis.moveToPoint(10, 25, 5000, {.forwards = false, .minSpeed = 100}, false);
    doinker.set_value(false);
    pros::delay(500);
    chassis.turnToHeading(-160, 1000);
    chassis.moveToPoint(10, 42, 2000, {.forwards = false, .maxSpeed = 60, .minSpeed = 40}, false);
    clamp.set_value(true);
    spinTop = true;
    pros::delay(200);
    chassis.turnToHeading(180, 1000, {}, false);
    chassis.moveToPoint(20, 10, 1500, {.forwards = false}, false);
    clamp.set_value(false);
    chassis.moveToPoint(20, 43, 2000);
    pros::delay(300);
    chassis.turnToHeading(-90, 1000);

    chassis.moveToPoint(50, 43, 2000, {.forwards = false, .maxSpeed = 40}, false);
    clamp.set_value(true);
    pros::delay(300);
    chassis.moveToPoint(-5, -10, 1500);
    pros::delay(300);
    chassis.moveToPoint(10, 10, 1000, {.forwards = false});
}

void autonomous(){
    // intakeAutonControl.resume();
    // armAutonControl.resume();
    // armTelopControl.suspend();
    // intakeTelopControl.suspend();
    pros::Task intakeAutonControl(intakeAuton);
    pros::Task armAutonControl(armAuton);

    armRotation.set_position(8500);

    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    skills();
    // prime = true;

    // goalSideBlue();
    // ringSideBlue();
    // goalRushBlue();

    // goalSideRed();
    // ringSideRed();
}