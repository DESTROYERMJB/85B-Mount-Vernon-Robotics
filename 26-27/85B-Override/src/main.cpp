#include "main.h"
#include "pros/llemu.hpp"
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

void opcontrol() {
    while (true) {
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.move(127);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            lift.move(-127);
        } else {
            lift.move(0);
        }
        chassis.driverControl(master);
        pros::delay(10);
        
    }

}
