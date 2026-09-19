#include "Chassis.hpp"
#include "lemlib/config.hpp"
#include "lemlib/MotionHandler.hpp"
#include "lemlib/Timer.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"

// lemlib/config.hpp globals: placeholders until the Chassis constructor fills them in from its settings
lemlib::PID angular_pid(0, 0, 0);
lemlib::PID lateral_pid(0, 0, 0);
std::function<units::Pose()> pose_getter = [] { return units::Pose(); };
lemlib::MotorGroup* left_motors = nullptr;
lemlib::MotorGroup* right_motors = nullptr;
lemlib::ExitConditionGroup<AngleRange> angular_exit_conditions({});
lemlib::ExitConditionGroup<Length> lateral_exit_conditions({});
Length track_width = 0_in;
Number drift_compensation = 0;
Number angular_slew = 0;
Number lateral_slew = 0;

namespace lemlib {

// pros::Controller::get_analog's full range, used to scale raw joystick input to -1..+1
static constexpr Number CONTROLLER_ANALOG_MAX = 127;

Chassis::Chassis(Drivetrain drivetrain, LateralController lateralController, AngularController angularController,
                 OdomSensors sensors, DriveType defaultDriveType, DriveCurve throttleCurve, DriveCurve turnCurve)
    : m_odom({&sensors.imu()}, {&sensors.verticalTrackingWheel()}, {&sensors.horizontalTrackingWheel()}),
      m_imus({&sensors.imu()}),
      m_drivetrain(drivetrain),
      m_lateralController(lateralController),
      m_angularController(angularController),
      m_defaultDriveType(defaultDriveType),
      m_throttleCurve(throttleCurve),
      m_turnCurve(turnCurve) {
    left_motors = &m_drivetrain.leftMotors;
    right_motors = &m_drivetrain.rightMotors;
    angular_pid = angularController.pid();
    lateral_pid = lateralController.pid();
    pose_getter = [this] { return getPose(); };
    angular_exit_conditions = angularController.exitConditions();
    lateral_exit_conditions = lateralController.exitConditions();
    track_width = drivetrain.trackWidth;
    drift_compensation = drivetrain.horizontalDrift;
    angular_slew = angularController.slew;
    lateral_slew = lateralController.slew;
}

void Chassis::calibrate() {
    for (IMU* imu : m_imus) imu->calibrate();
    // poll for calibration to finish instead of a blind fixed delay, bounded so a disconnected
    // or broken IMU can't hang initialize() forever
    Timer timer(3_sec);
    bool calibrating = true;
    while (calibrating && !timer.isDone()) {
        calibrating = false;
        for (IMU* imu : m_imus) {
            if (imu->isCalibrating() != 0) calibrating = true;
        }
        pros::delay(10);
    }
    m_odom.startTask();
}

units::Pose Chassis::getPose() { return m_odom.getPose(); }

double Chassis::getX() { return unit_config::toLength(getPose().x); }

double Chassis::getY() { return unit_config::toLength(getPose().y); }

double Chassis::getHeading() { return unit_config::toHeading(getPose().orientation); }

void Chassis::setPose(double x, double y, double heading) {
    m_odom.setPose({unit_config::length(x), unit_config::length(y), unit_config::heading(heading)});
}

void Chassis::turnToHeading(double heading, double timeout, TurnToParams params, bool async,
                            std::optional<uint32_t> priority) {
    const std::variant<Angle, units::V2Position> target = unit_config::heading(heading);
    const Time time = unit_config::time(timeout);
    if (async) {
        motion_handler::move([target, time, params] { lemlib::turnTo(target, time, params, {}); }, priority);
    } else {
        lemlib::turnTo(target, time, params, {});
    }
}

void Chassis::turnToPoint(double x, double y, double timeout, TurnToParams params, bool async,
                          std::optional<uint32_t> priority) {
    const std::variant<Angle, units::V2Position> target = units::V2Position(unit_config::length(x), unit_config::length(y));
    const Time time = unit_config::time(timeout);
    if (async) {
        motion_handler::move([target, time, params] { lemlib::turnTo(target, time, params, {}); }, priority);
    } else {
        lemlib::turnTo(target, time, params, {});
    }
}

void Chassis::moveToPoint(double x, double y, double timeout, MoveToPointParams params, bool async,
                          std::optional<uint32_t> priority) {
    const units::V2Position target(unit_config::length(x), unit_config::length(y));
    const Time time = unit_config::time(timeout);
    if (async) {
        motion_handler::move([target, time, params] { lemlib::moveToPoint(target, time, params, {}); }, priority);
    } else {
        lemlib::moveToPoint(target, time, params, {});
    }
}

void Chassis::moveToPose(double x, double y, double heading, double timeout, MoveToPoseParams params, bool async,
                         std::optional<uint32_t> priority) {
    const units::Pose target(unit_config::length(x), unit_config::length(y), unit_config::heading(heading));
    const Time time = unit_config::time(timeout);
    if (async) {
        motion_handler::move([target, time, params] { lemlib::moveToPose(target, time, params, {}); }, priority);
    } else {
        lemlib::moveToPose(target, time, params, {});
    }
}

void Chassis::follow(const asset& path, double lookaheadDistance, double timeout, FollowParams params, bool async,
                     std::optional<uint32_t> priority) {
    const Length lookahead = unit_config::length(lookaheadDistance);
    const Time time = unit_config::time(timeout);
    if (async) {
        motion_handler::move([path, lookahead, time, params] { lemlib::follow(path, lookahead, time, params, {}); },
                             priority);
    } else {
        lemlib::follow(path, lookahead, time, params, {});
    }
}

bool Chassis::isMoving() const { return motion_handler::isMoving(); }

void Chassis::cancel() { motion_handler::cancel(); }

void Chassis::waitUntilDone() {
    while (motion_handler::isMoving()) pros::delay(5);
}

void Chassis::tank(Number left, Number right, bool disableDriveCurve) {
    const Number l = left / CONTROLLER_ANALOG_MAX;
    const Number r = right / CONTROLLER_ANALOG_MAX;
    m_drivetrain.leftMotors.move(disableDriveCurve ? l : m_throttleCurve(l));
    m_drivetrain.rightMotors.move(disableDriveCurve ? r : m_throttleCurve(r));
}

void Chassis::arcade(Number throttle, Number turn, bool disableDriveCurve) {
    const Number t = throttle / CONTROLLER_ANALOG_MAX;
    const Number s = turn / CONTROLLER_ANALOG_MAX;
    const Number curvedThrottle = disableDriveCurve ? t : m_throttleCurve(t);
    const Number curvedTurn = disableDriveCurve ? s : m_turnCurve(s);
    const DriveOutputs outputs = desaturate(curvedThrottle, curvedTurn);
    m_drivetrain.leftMotors.move(outputs.left);
    m_drivetrain.rightMotors.move(outputs.right);
}

void Chassis::curvature(Number throttle, Number turn, bool disableDriveCurve) {
    const Number t = throttle / CONTROLLER_ANALOG_MAX;
    const Number s = turn / CONTROLLER_ANALOG_MAX;
    const Number curvedThrottle = disableDriveCurve ? t : m_throttleCurve(t);
    const Number curvedTurn = disableDriveCurve ? s : m_turnCurve(s);
    const DriveOutputs outputs = desaturate(curvedThrottle, curvedTurn * units::abs(curvedThrottle));
    m_drivetrain.leftMotors.move(outputs.left);
    m_drivetrain.rightMotors.move(outputs.right);
}

void Chassis::driverControl(pros::Controller& controller, bool disableDriveCurve) {
    switch (m_defaultDriveType) {
        case DriveType::TANK:
            tank(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                 controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y), disableDriveCurve);
            break;
        case DriveType::ARCADE:
            arcade(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                   controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), disableDriveCurve);
            break;
        case DriveType::CURVATURE:
            curvature(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y),
                      controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X), disableDriveCurve);
            break;
    }
}
} // namespace lemlib
