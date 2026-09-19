#pragma once

#include "Chassis.hpp"

/**
 * the robot's single Chassis instance, wiring together the drivetrain, odometry, and motion
 * defaults (defined in robot.cpp, which also defines the lemlib/config.hpp globals)
 */
extern lemlib::Chassis chassis;
