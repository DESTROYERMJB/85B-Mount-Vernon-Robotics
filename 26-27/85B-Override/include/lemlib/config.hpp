#pragma once

// this file is used to configure default values used by motion algorithms used in LemLib

#include "ExitCondition.hpp"
#include "PID.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "units/Pose.hpp"
#include <functional>

// defined in Chassis.cpp and filled in by the lemlib::Chassis constructor

extern lemlib::PID angular_pid;
extern lemlib::PID lateral_pid;

extern std::function<units::Pose()> pose_getter;

// the drivetrain's motor groups, owned by the Chassis's Drivetrain
extern lemlib::MotorGroup* left_motors;
extern lemlib::MotorGroup* right_motors;

extern lemlib::ExitConditionGroup<AngleRange> angular_exit_conditions;
extern lemlib::ExitConditionGroup<Length> lateral_exit_conditions;

extern Length track_width;

extern Number drift_compensation;

extern Number angular_slew;
extern Number lateral_slew;