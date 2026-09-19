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
            const units::Pose p = chassis.getPose();
            pros::lcd::print(0, "X: %f", to_in(p.x));
            pros::lcd::print(1, "Y: %f", to_in(p.y));
            pros::lcd::print(2, "Theta: %f", to_cDeg(p.orientation));
            pros::delay(10);
        }
    });
}

void disabled() {}

void autonomous() {
    // example motion, wire up the real routine here
    chassis.turnTo(90_cDeg, 2_sec);
    chassis.waitUntilDone();
}

void opcontrol() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    while (true) {
        chassis.driverControl(master);
        pros::delay(10);
    }
}
