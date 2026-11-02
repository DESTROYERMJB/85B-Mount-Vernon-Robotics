#pragma once

#include "Chassis.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"

/** the driver controller (defined in robot.cpp) */
extern pros::Controller master;

/** the lift motors (defined in robot.cpp) */
extern pros::MotorGroup lift;

/**
 * the robot's single Chassis instance, wiring together the drivetrain, odometry, and motion
 * defaults (defined in robot.cpp, which also defines the lemlib/config.hpp globals)
 */
extern lemlib::Chassis chassis;
