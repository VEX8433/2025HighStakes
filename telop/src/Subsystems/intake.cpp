#include "Intake.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"

Intake::Intake(pros::Motor bottom, pros::Motor top, pros::v5::Optical optical) : bottom(bottom), top(top), optical(optical){}


void Intake::telOP(bool R1, bool R2){
    if(R1 && R2){
        bottom.move_velocity(-200);
        top.move_velocity(-200);
    }
    else if(R1){
        bool thing = false;
        bottom.move_velocity(200);
        top.move_velocity(200);
        if(optical.get_hue() < 240 && optical.get_hue() > 200){
            thing = true;
        }
        pros::delay(20);
        if(thing){
            top.move_velocity(-1);
            pros::delay(200);
            top.move_velocity(200);
        }
        pros::delay(70);
        if(top.get_actual_velocity() < 20){
            top.move_velocity(-200);
            pros::delay(150);
            top.move_velocity(200);
        }
    }
    else if(R2){
        top.move_velocity(0);
        bottom.move_velocity(200);
    } 
    else{
        top.move_velocity(0);
        bottom.move_velocity(0);
    }
}