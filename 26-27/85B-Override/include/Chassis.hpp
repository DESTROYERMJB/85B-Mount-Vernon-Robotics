#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <variant>
#include <vector>
#include "hardware/IMU/IMU.hpp"
#include "hardware/Motor/MotorGroup.hpp"
#include "hot-cold-asset/asset.hpp"
#include "lemlib/ExitCondition.hpp"
#include "lemlib/PID.hpp"
#include "pros/misc.hpp"
#include "lemlib/motions/follow.hpp"
#include "lemlib/motions/moveToPoint.hpp"
#include "lemlib/motions/moveToPose.hpp"
#include "lemlib/motions/turnTo.hpp"
#include "lemlib/tracking/TrackingWheelOdom.hpp"
#include "units/Pose.hpp"
#include "units/units.hpp"
#include "UnitConfig.hpp"

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
    /** Left stick controls the left side, right stick controls the right side. */
    TANK,

    /** One stick's axis is throttle, the other (or the same stick's other axis) is turning. */
    ARCADE,

    /** Throttle plus a turn radius, similar to arcade but curves instead of pivots at low throttle. */
    CURVATURE
};


/**
 * @brief the sensors used for tracking-wheel odometry
 *
 * The sensors are referenced, not owned, so they must outlive the OdomSensors (and the Chassis).
 *
 * @b Example:
 * @code {.cpp}
 * lemlib::V5InertialSensor imu(1);
 * lemlib::TrackingWheel verticalTracker(2, 2.75_in, 0_in);
 * lemlib::TrackingWheel horizontalTracker(-3, 2.75_in, 0_in);
 * lemlib::OdomSensors odomSensors(verticalTracker, horizontalTracker, imu);
 * @endcode
 */
class OdomSensors {
    public:
        /**
         * @param verticalTrackingWheel the tracking wheel measuring forward/backward travel
         * @param horizontalTrackingWheel the tracking wheel measuring sideways travel
         * @param imu the inertial sensor used for heading
         */
        OdomSensors(TrackingWheel& verticalTrackingWheel, TrackingWheel& horizontalTrackingWheel, IMU& imu)
            : m_vertical(verticalTrackingWheel),
              m_horizontal(horizontalTrackingWheel),
              m_imu(imu) {}

        TrackingWheel& verticalTrackingWheel() const { return m_vertical; }
        TrackingWheel& horizontalTrackingWheel() const { return m_horizontal; }
        IMU& imu() const { return m_imu; }
    private:
        TrackingWheel& m_vertical;
        TrackingWheel& m_horizontal;
        IMU& m_imu;
};

/**
 * @brief the physical drivetrain: motor groups and geometry
 *
 * @b Example:
 * The Drivetrain owns its motor groups, so they can be constructed inline:
 * @code {.cpp}
 * // track width 12 in, wheel size 3.25 in, 360 rpm (units are set in UnitConfig.hpp)
 * lemlib::Drivetrain drivetrain(lemlib::MotorGroup({1, -2, 3}, lemlib::Cartridge::BLUE),
 *                               lemlib::MotorGroup({4, 5, -6}, lemlib::Cartridge::BLUE), 12, 3.25, 360, 8);
 * @endcode
 */
struct Drivetrain {
        /**
         * @param left the left motor group
         * @param right the right motor group
         * @param trackWidth distance between the left and right wheels' contact patches (standard length unit)
         * @param wheelSize the drive wheel diameter (standard length unit)
         * @param outputRpm the drivetrain's output rpm, after any external gearing
         * @param horizontalDrift drift compensation (2 for non-traction wheels, 8 for traction wheels is a
         * good starting point)
         */
        Drivetrain(MotorGroup left, MotorGroup right, double trackWidth, double wheelSize, double outputRpm,
                   double horizontalDrift)
            : leftMotors(left),
              rightMotors(right),
              trackWidth(unit_config::length(trackWidth)),
              wheelSize(unit_config::length(wheelSize)),
              outputRpm(unit_config::rpm(outputRpm)),
              horizontalDrift(horizontalDrift) {}

        MotorGroup leftMotors;
        MotorGroup rightMotors;
        Length trackWidth;
        Length wheelSize;
        AngularVelocity outputRpm;
        Number horizontalDrift;
};

/**
 * @brief PID gains, exit conditions, and slew for a lateral (driving) controller
 */
struct LateralController {
        /**
         * @param kP proportional gain
         * @param kI integral gain
         * @param kD derivative gain
         * @param antiWindup integral is reset while error is outside this range (0 disables)
         * @param smallError error range for the small-error exit condition (standard length unit)
         * @param smallErrorTimeout time within smallError before exiting (standard time unit)
         * @param largeError error range for the large-error exit condition (standard length unit)
         * @param largeErrorTimeout time within largeError before exiting (standard time unit)
         * @param slew maximum acceleration (0 disables)
         */
        LateralController(double kP, double kI, double kD, double antiWindup, double smallError,
                          double smallErrorTimeout, double largeError, double largeErrorTimeout, double slew)
            : kP(kP),
              kI(kI),
              kD(kD),
              antiWindup(antiWindup),
              smallError(unit_config::length(smallError)),
              smallErrorTimeout(unit_config::time(smallErrorTimeout)),
              largeError(unit_config::length(largeError)),
              largeErrorTimeout(unit_config::time(largeErrorTimeout)),
              slew(slew) {}

        PID pid() const { return PID(kP, kI, kD, antiWindup); }

        ExitConditionGroup<Length> exitConditions() const {
            return ExitConditionGroup<Length>(
                {ExitCondition<Length>(smallError, smallErrorTimeout), ExitCondition<Length>(largeError, largeErrorTimeout)});
        }

        Number kP, kI, kD, antiWindup;
        Length smallError;
        Time smallErrorTimeout;
        Length largeError;
        Time largeErrorTimeout;
        Number slew;
};

/**
 * @brief PID gains, exit conditions, and slew for an angular (turning) controller
 */
struct AngularController {
        /** @see LateralController - identical parameters, but error ranges are angles (standard angle unit) */
        AngularController(double kP, double kI, double kD, double antiWindup, double smallError,
                          double smallErrorTimeout, double largeError, double largeErrorTimeout, double slew)
            : kP(kP),
              kI(kI),
              kD(kD),
              antiWindup(antiWindup),
              smallError(unit_config::angle(smallError)),
              smallErrorTimeout(unit_config::time(smallErrorTimeout)),
              largeError(unit_config::angle(largeError)),
              largeErrorTimeout(unit_config::time(largeErrorTimeout)),
              slew(slew) {}

        PID pid() const { return PID(kP, kI, kD, antiWindup); }

        ExitConditionGroup<AngleRange> exitConditions() const {
            return ExitConditionGroup<AngleRange>({ExitCondition<AngleRange>(smallError, smallErrorTimeout),
                                                   ExitCondition<AngleRange>(largeError, largeErrorTimeout)});
        }

        Number kP, kI, kD, antiWindup;
        AngleRange smallError;
        Time smallErrorTimeout;
        AngleRange largeError;
        Time largeErrorTimeout;
        Number slew;
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
         * Also publishes the controller/drivetrain settings to the lemlib/config.hpp globals the motion
         * algorithms use as defaults, so nothing else needs to define them.
         *
         * @param drivetrain the motor groups and drivetrain geometry
         * @param lateralController PID, exit conditions, and slew for driving
         * @param angularController PID, exit conditions, and slew for turning
         * @param sensors the sensors to use for odometry
         * @param defaultDriveType which driver-control scheme driverControl() should use. Defaults to ARCADE
         * @param throttleCurve curve applied to the throttle input during driver control. Defaults to no curve
         * @param turnCurve curve applied to the turn input during driver control. Defaults to no curve
         *
         * @b Example:
         * @code {.cpp}
         * lemlib::Chassis chassis(drivetrain, lateralController, angularController, odomSensors,
         *                         lemlib::DriveType::ARCADE);
         * @endcode
         */
        Chassis(Drivetrain drivetrain, LateralController lateralController, AngularController angularController,
                OdomSensors sensors, DriveType defaultDriveType = DriveType::ARCADE,
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
        /** @brief the robot's estimated x position, in the standard length unit */
        double getX();
        /** @brief the robot's estimated y position, in the standard length unit */
        double getY();
        /** @brief the robot's estimated heading, in the standard heading unit */
        double getHeading();
        /**
         * @brief Set the estimated pose of the robot
         *
         * @param x x position (standard length unit)
         * @param y y position (standard length unit)
         * @param heading heading (standard heading unit)
         */
        void setPose(double x, double y, double heading);

        /**
         * @brief Turn the robot to face a heading
         *
         * @param heading the heading to face (standard heading unit)
         * @param timeout the maximum amount of time the motion can run for (standard time unit)
         * @param params struct containing parameters for the turn
         * @param async whether to run the motion in the background and return immediately (true, default) or block
         * until it finishes (false)
         * @param priority the priority to run the motion at, if async. Defaults to the calling task's priority
         */
        void turnToHeading(double heading, double timeout, TurnToParams params = {}, bool async = true,
                           std::optional<uint32_t> priority = std::nullopt);
        /**
         * @brief Turn the robot to face a point
         *
         * @param x the point's x position (standard length unit)
         * @param y the point's y position (standard length unit)
         * @param timeout the maximum amount of time the motion can run for (standard time unit)
         * @param params struct containing parameters for the turn
         * @param async whether to run the motion in the background and return immediately (true, default) or block
         * until it finishes (false)
         * @param priority the priority to run the motion at, if async. Defaults to the calling task's priority
         */
        void turnToPoint(double x, double y, double timeout, TurnToParams params = {}, bool async = true,
                         std::optional<uint32_t> priority = std::nullopt);
        /**
         * @brief Move the robot to a point
         *
         * @param x the target x position (standard length unit)
         * @param y the target y position (standard length unit)
         * @param timeout the maximum amount of time the motion can run for (standard time unit)
         * @param params struct containing parameters for the motion
         * @param async whether to run the motion in the background and return immediately (true, default) or block
         * until it finishes (false)
         * @param priority the priority to run the motion at, if async. Defaults to the calling task's priority
         */
        void moveToPoint(double x, double y, double timeout, MoveToPointParams params = {}, bool async = true,
                         std::optional<uint32_t> priority = std::nullopt);
        /**
         * @brief Move the robot to a pose
         *
         * @param x the target x position (standard length unit)
         * @param y the target y position (standard length unit)
         * @param heading the target heading (standard heading unit)
         * @param timeout the maximum amount of time the motion can run for (standard time unit)
         * @param params struct containing parameters for the motion
         * @param async whether to run the motion in the background and return immediately (true, default) or block
         * until it finishes (false)
         * @param priority the priority to run the motion at, if async. Defaults to the calling task's priority
         */
        void moveToPose(double x, double y, double heading, double timeout, MoveToPoseParams params = {},
                        bool async = true, std::optional<uint32_t> priority = std::nullopt);
        /**
         * @brief Follow a path
         *
         * @param path the path to follow
         * @param lookaheadDistance the lookahead distance for the pure pursuit algorithm (standard length unit)
         * @param timeout the maximum amount of time the motion can run for (standard time unit)
         * @param params struct containing parameters for the motion
         * @param async whether to run the motion in the background and return immediately (true, default) or block
         * until it finishes (false)
         * @param priority the priority to run the motion at, if async. Defaults to the calling task's priority
         */
        void follow(const asset& path, double lookaheadDistance, double timeout, FollowParams params = {},
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
        Drivetrain m_drivetrain;
        LateralController m_lateralController;
        AngularController m_angularController;
        DriveType m_defaultDriveType;
        DriveCurve m_throttleCurve;
        DriveCurve m_turnCurve;
};
} // namespace lemlib
