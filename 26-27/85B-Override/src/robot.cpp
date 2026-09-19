#include "robot.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "lemlib/config.hpp"

// drivetrain motors. These are the same objects lemlib::config.hpp's left_motors/right_motors
// externs refer to, and the same objects Chassis holds references to - single source of truth,
// no duplicate MotorGroup construction.
// TODO: confirm these ports/gearing are 85B-Override's final drivetrain configuration
lemlib::MotorGroup left_motors({-1, 11, -12, 13}, 360_rpm);
lemlib::MotorGroup right_motors({8, 10}, 360_rpm);

// odometry sensors
// TODO: confirm these ports, offsets, and wheel diameter are correct final measurements
static lemlib::V5InertialSensor imu(1);
static lemlib::TrackingWheel verticalTracker({'E', 'F'}, true, 2.75_in, 26.5_cm / 2);
static lemlib::TrackingWheel horizontalTracker({'G', 'H'}, false, 2.75_in, -26.5_cm / 2);

// the robot
lemlib::Chassis chassis(
    {.imus = {&imu}, .verticalWheels = {&verticalTracker}, .horizontalWheels = {&horizontalTracker}},
    {.leftMotors = left_motors, .rightMotors = right_motors},
    lemlib::DriveType::TANK
);

// lemlib/config.hpp defaults - used by lemlib::turnTo/moveToPoint/moveToPose/follow whenever
// Chassis (or any other caller) constructs their Settings struct with {}. Values carried over
// from main.cpp's previous placeholders where one existed; everything else is a conservative
// starting point that needs tuning on the real robot.
const lemlib::PID angular_pid(0.05, 0, 0); // TODO: tune
const lemlib::PID lateral_pid(0, 0, 0); // TODO: tune
const std::function<units::Pose()> pose_getter = [] { return chassis.getPose(); };
const lemlib::ExitConditionGroup<AngleRange> angular_exit_conditions({lemlib::ExitCondition<AngleRange>(1_stDeg, 2_sec)}
);
const lemlib::ExitConditionGroup<Length> lateral_exit_conditions({lemlib::ExitCondition<Length>(1_in, 200_msec)}
); // TODO: tune
const Length track_width = 26.5_cm; // TODO: measure the real track width (left wheel to right wheel contact patch)
const Number drift_compensation = 0.5; // TODO: tune (2 for non-traction wheels, 8 for traction wheels is LemLib's old rule of thumb)
const Number angular_slew = 0; // disabled by default
const Number lateral_slew = 0; // disabled by default
