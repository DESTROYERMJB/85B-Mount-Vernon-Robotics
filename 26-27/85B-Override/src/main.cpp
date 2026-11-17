#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "robot.hpp"


void initialize() {
    pros::lcd::initialize();
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    chassis.calibrate();
}

void disabled() {}

void autonomous() {
    // example motion, wire up the real routine here
    chassis.setPose(0,0,90);
    chassis.moveToPose(24, 24, 90, 2000); // x (in), y (in), heading (deg), timeout (ms)
}

bool matchLoad = false;
bool blocking = false;

void frontInteraction() {
    while (true) {
        if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && !matchLoad) {
            if(blocking) {
                blockerPiston.retract();
                blocking = false;
                pros::delay(50);
            }
            intakePiston.extend();
            matchLoad = true;
            pros::delay(100);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && matchLoad) {
            intakePiston.retract();
            matchLoad = false;
            pros::delay(100);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && !blocking) {
            blockerPiston.extend();
            blocking = true;
            pros::delay(100);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && blocking) {
            blockerPiston.retract();
            blocking = false;
            pros::delay(100);
        }
        pros::delay(10);
    }
}

bool wristExtended = false;
bool clawClamped = false;

void placement() {
    while (true) {
        if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_X) && !wristExtended) {
            wrist.extend();
            wristExtended = true;
            pros::delay(100);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_X) && wristExtended) {
            wrist.retract();
            wristExtended = false;
            pros::delay(100);
        }
        if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_A) && !clawClamped) {
            claw.extend();
            clawClamped = true;
            pros::delay(100);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_A) && clawClamped) {
            claw.retract();
            clawClamped = false;
            pros::delay(100);
        }
        pros::delay(10);
    }
}

void opcontrol() {
    pros::Task frontInteractionTask(frontInteraction);
    pros::Task placementTask(placement);
    while (true) {
        if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.move(127);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            lift.move(-127);
        } else {
            lift.move(0);
        }
        if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake.move(127);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake.move(-127);
        } else {
            intake.move(0);
        }
        chassis.driverControl(driveController);
        pros::delay(10);
        
    }

}
