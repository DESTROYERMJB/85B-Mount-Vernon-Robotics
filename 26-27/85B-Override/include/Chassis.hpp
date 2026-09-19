#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <variant>
#include <vector>
#include "hardware/IMU/IMU.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "hot-cold-asset/asset.hpp"
#include "pros/misc.hpp"
#include "lemlib/motions/follow.hpp"
#include "lemlib/motions/moveToPoint.hpp"
#include "lemlib/motions/moveToPose.hpp"
#include "lemlib/motions/turnTo.hpp"
#include "lemlib/tracking/TrackingWheelOdom.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"

namespace lemlib {

/**
 * @brief a function that reshapes a raw driver-control input (-1 to +1) before it's used
 *
 * Identity (`[](Number x) { return x; }`) means no curve is applied - the raw input is passed
 * straight through
 */
using DriveCurve = std::function<Number(Number)>;

/**
 * @brief which driver-control scheme Chassis::driverControl should use by default
 */
enum class DriveType {
    TANK, /** left stick controls the left side, right stick controls the right side */
    ARCADE, /** one stick's axis is throttle, the other (or the same stick's other axis) is turning */
    CURVATURE /** throttle plus a turn radius, similar to arcade but curves instead of pivots at low throttle */
};

/**
 * @brief the sensors used for tracking-wheel odometry
 *
 * Grouped into their own struct (rather than 3 separate constructor params) so a Chassis
 * declaration is self-documenting via designated initializers, e.g.
 * `{.imus = {&imu}, .verticalWheels = {&vertical}}` - matches old LemLib's OdomSensors naming.
 * Any field can be left empty (`{}`) if that sensor type isn't used.
 */
struct OdomSensors {
        std::vector<IMU*> imus = {};
        std::vector<TrackingWheel*> verticalWheels = {};
        std::vector<TrackingWheel*> horizontalWheels = {};
};

/**
 * @brief the drivetrain's motor groups
 *
 * Grouped into its own struct for the same reason as OdomSensors - matches old LemLib's
 * Drivetrain naming.
 */
struct Drivetrain {
        MotorGroup& leftMotors;
        MotorGroup& rightMotors;
};

/**
 * @brief Chassis class
 *
 * Ties together a drivetrain, tracking-wheel odometry, and the LemLib motion algorithms
 * (turnTo, moveToPoint, moveToPose, follow) into a single, ergonomic object, so call sites
 * don't have to hand-build a *Settings struct (referencing the config.hpp globals) on every
 * call. Motion parameters are passed through to the underlying free functions unchanged.
 */
class Chassis {
    public:
        /**
         * @brief Construct a new Chassis object
         *
         * @param sensors the sensors to use for odometry
         * @param drivetrain the drivetrain's left and right motor groups
         * @param defaultDriveType which driver-control scheme driverControl() should use. Defaults to ARCADE
         * @param throttleCurve curve applied to the throttle input during driver control. Defaults to no curve
         * @param turnCurve curve applied to the turn input during driver control. Defaults to no curve
         *
         * @b Example:
         * @code {.cpp}
         * lemlib::Chassis chassis(
         *     {.imus = {&imu}, .verticalWheels = {&verticalTracker}, .horizontalWheels = {&horizontalTracker}},
         *     {.leftMotors = left_motors, .rightMotors = right_motors},
         *     lemlib::DriveType::ARCADE
         * );
         * @endcode
         */
        Chassis(OdomSensors sensors, Drivetrain drivetrain, DriveType defaultDriveType = DriveType::ARCADE,
                DriveCurve throttleCurve = [](Number x) { return x; },
                DriveCurve turnCurve = [](Number x) { return x; });

        /**
         * @brief calibrate the chassis. This should be called in the initialize function
         *
         * Calibrates every IMU passed to the constructor (polling isCalibrating() rather than a fixed delay)
         * and starts the odometry tracking task.
         */
        void calibrate();
        /**
         * @brief Get the estimated pose of the robot
         */
        units::Pose getPose();
        /**
         * @brief Set the estimated pose of the robot
         */
        void setPose(units::Pose pose);

        /**
         * @brief Turn the robot to face a heading or position
         *
         * @param target the target to turn to. Can be an angle, or a position
         * @param timeout the maximum amount of time the motion can run for
         * @param params struct containing parameters for the turn
         * @param async whether to run the motion in the background and return immediately (true, default) or block
         * until it finishes (false)
         * @param priority the priority to run the motion at, if async. Defaults to the calling task's priority
         */
        void turnTo(std::variant<Angle, units::V2Position> target, Time timeout, TurnToParams params = {},
                    bool async = true, std::optional<uint32_t> priority = std::nullopt);
        /**
         * @brief Move the robot to a point
         *
         * @param target the target point
         * @param timeout the maximum amount of time the motion can run for
         * @param params struct containing parameters for the motion
         * @param async whether to run the motion in the background and return immediately (true, default) or block
         * until it finishes (false)
         * @param priority the priority to run the motion at, if async. Defaults to the calling task's priority
         */
        void moveToPoint(units::V2Position target, Time timeout, MoveToPointParams params = {}, bool async = true,
                         std::optional<uint32_t> priority = std::nullopt);
        /**
         * @brief Move the robot to a pose
         *
         * @param target the target pose
         * @param timeout the maximum amount of time the motion can run for
         * @param params struct containing parameters for the motion
         * @param async whether to run the motion in the background and return immediately (true, default) or block
         * until it finishes (false)
         * @param priority the priority to run the motion at, if async. Defaults to the calling task's priority
         */
        void moveToPose(units::Pose target, Time timeout, MoveToPoseParams params = {}, bool async = true,
                        std::optional<uint32_t> priority = std::nullopt);
        /**
         * @brief Follow a path
         *
         * @param path the path to follow
         * @param lookaheadDistance the lookahead distance for the pure pursuit algorithm
         * @param timeout the maximum amount of time the motion can run for
         * @param params struct containing parameters for the motion
         * @param async whether to run the motion in the background and return immediately (true, default) or block
         * until it finishes (false)
         * @param priority the priority to run the motion at, if async. Defaults to the calling task's priority
         */
        void follow(const asset& path, Length lookaheadDistance, Time timeout, FollowParams params = {},
                    bool async = true, std::optional<uint32_t> priority = std::nullopt);

        /**
         * @brief whether a motion is currently running
         */
        bool isMoving() const;
        /**
         * @brief cancel the currently running motion, if any
         */
        void cancel();
        /**
         * @brief block until the currently running motion (if any) has finished
         */
        void waitUntilDone();

        /**
         * @brief drive the robot using tank controls
         *
         * @param left raw left-side joystick value, from -127 to +127 (e.g. straight from
         * pros::Controller::get_analog) - scaled to -1..+1 internally
         * @param right raw right-side joystick value, from -127 to +127
         * @param disableDriveCurve whether to bypass the configured throttle/turn curves. False by default
         */
        void tank(Number left, Number right, bool disableDriveCurve = false);
        /**
         * @brief drive the robot using arcade controls
         *
         * @param throttle raw forward/backward joystick value, from -127 to +127 (e.g. straight from
         * pros::Controller::get_analog) - scaled to -1..+1 internally
         * @param turn raw turning joystick value, from -127 to +127
         * @param disableDriveCurve whether to bypass the configured throttle/turn curves. False by default
         */
        void arcade(Number throttle, Number turn, bool disableDriveCurve = false);
        /**
         * @brief drive the robot using curvature controls
         *
         * @param throttle raw forward/backward joystick value, from -127 to +127 (e.g. straight from
         * pros::Controller::get_analog) - scaled to -1..+1 internally
         * @param turn raw turning joystick value, from -127 to +127
         * @param disableDriveCurve whether to bypass the configured throttle/turn curves. False by default
         */
        void curvature(Number throttle, Number turn, bool disableDriveCurve = false);
        /**
         * @brief drive the robot using whichever scheme was set as the default at construction
         *
         * Reads the correct joystick axes for the configured DriveType (TANK: left Y + right Y; ARCADE/CURVATURE:
         * left Y for throttle + right X for turn) and forwards to tank()/arcade()/curvature() accordingly, so a
         * call site never needs to know or care which scheme is active - switching DriveType at construction is
         * the only change needed.
         *
         * @param controller the controller to read joystick input from
         * @param disableDriveCurve whether to bypass the configured throttle/turn curves. False by default
         */
        void driverControl(pros::Controller& controller, bool disableDriveCurve = false);
    private:
        TrackingWheelOdometry m_odom;
        std::vector<IMU*> m_imus;
        MotorGroup& m_leftMotors;
        MotorGroup& m_rightMotors;
        DriveType m_defaultDriveType;
        DriveCurve m_throttleCurve;
        DriveCurve m_turnCurve;
};
} // namespace lemlib
