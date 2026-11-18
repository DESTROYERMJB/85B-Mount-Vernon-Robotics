#pragma once

#include "Chassis.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/adi.hpp"
#include "pros/vision.hpp"
#include "pros/distance.hpp"

/** the driver controller (defined in robot.cpp) */
extern pros::Controller driveController;
/** the partner controller (defined in robot.cpp) */
extern pros::Controller liftController;

/** the lift motors (defined in robot.cpp) */
extern pros::MotorGroup lift;
extern pros::Motor intake;

extern pros::adi::Pneumatics intakePiston;
extern pros::adi::Pneumatics blockerPiston;
extern pros::adi::Pneumatics wrist;
extern pros::adi::Pneumatics claw;

extern pros::Distance leftDistance;
extern pros::Distance rightDistance;

extern pros::Vision leftColor;
extern pros::Vision rightColor;

/**
 * the robot's single Chassis instance, wiring together the drivetrain, odometry, and motion
 * defaults (defined in robot.cpp, which also defines the lemlib/config.hpp globals)
 */
extern lemlib::Chassis chassis;
