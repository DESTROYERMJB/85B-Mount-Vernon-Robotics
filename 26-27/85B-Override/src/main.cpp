#include "main.h"
#include "lemlog/logger/sinks/terminal.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"
#include "robot.hpp"

logger::Terminal terminal;

void initialize() {
    terminal.setLoggingLevel(logger::Level::DEBUG);
    pros::lcd::initialize();

    chassis.calibrate();
    pros::Task([] {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getX());
            pros::lcd::print(1, "Y: %f", chassis.getY());
            pros::lcd::print(2, "Theta: %f", chassis.getHeading());
            pros::delay(10);
        }
    });
}

void disabled() {}

void autonomous() {
    // example motion, wire up the real routine here
    chassis.setPose(0,0,90);
    chassis.moveToPose(24, 24, 90, 2000); // x (in), y (in), heading (deg), timeout (ms)
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    while (true) {
        chassis.driverControl(master);
        pros::delay(10);
    }
}
