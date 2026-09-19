#include "Chassis.hpp"
#include "lemlib/MotionHandler.hpp"
#include "lemlib/Timer.hpp"
#include "lemlib/util.hpp"
#include "pros/rtos.hpp"

namespace lemlib {

// pros::Controller::get_analog's full range, used to scale raw joystick input to -1..+1
static constexpr Number CONTROLLER_ANALOG_MAX = 127;

Chassis::Chassis(OdomSensors sensors, Drivetrain drivetrain, DriveType defaultDriveType, DriveCurve throttleCurve,
                 DriveCurve turnCurve)
    : m_odom(sensors.imus, sensors.verticalWheels, sensors.horizontalWheels),
      m_imus(sensors.imus),
      m_leftMotors(drivetrain.leftMotors),
      m_rightMotors(drivetrain.rightMotors),
      m_defaultDriveType(defaultDriveType),
      m_throttleCurve(throttleCurve),
      m_turnCurve(turnCurve) {}

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

void Chassis::setPose(units::Pose pose) { m_odom.setPose(pose); }

void Chassis::turnTo(std::variant<Angle, units::V2Position> target, Time timeout, TurnToParams params, bool async,
                     std::optional<uint32_t> priority) {
    if (async) {
        motion_handler::move([target, timeout, params] { lemlib::turnTo(target, timeout, params, {}); }, priority);
    } else {
        lemlib::turnTo(target, timeout, params, {});
    }
}

void Chassis::moveToPoint(units::V2Position target, Time timeout, MoveToPointParams params, bool async,
                          std::optional<uint32_t> priority) {
    if (async) {
        motion_handler::move([target, timeout, params] { lemlib::moveToPoint(target, timeout, params, {}); },
                             priority);
    } else {
        lemlib::moveToPoint(target, timeout, params, {});
    }
}

void Chassis::moveToPose(units::Pose target, Time timeout, MoveToPoseParams params, bool async,
                         std::optional<uint32_t> priority) {
    if (async) {
        motion_handler::move([target, timeout, params] { lemlib::moveToPose(target, timeout, params, {}); },
                             priority);
    } else {
        lemlib::moveToPose(target, timeout, params, {});
    }
}

void Chassis::follow(const asset& path, Length lookaheadDistance, Time timeout, FollowParams params, bool async,
                     std::optional<uint32_t> priority) {
    if (async) {
        motion_handler::move(
            [path, lookaheadDistance, timeout, params] { lemlib::follow(path, lookaheadDistance, timeout, params, {}); },
            priority);
    } else {
        lemlib::follow(path, lookaheadDistance, timeout, params, {});
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
    m_leftMotors.move(disableDriveCurve ? l : m_throttleCurve(l));
    m_rightMotors.move(disableDriveCurve ? r : m_throttleCurve(r));
}

void Chassis::arcade(Number throttle, Number turn, bool disableDriveCurve) {
    const Number t = throttle / CONTROLLER_ANALOG_MAX;
    const Number s = turn / CONTROLLER_ANALOG_MAX;
    const Number curvedThrottle = disableDriveCurve ? t : m_throttleCurve(t);
    const Number curvedTurn = disableDriveCurve ? s : m_turnCurve(s);
    const DriveOutputs outputs = desaturate(curvedThrottle, curvedTurn);
    m_leftMotors.move(outputs.left);
    m_rightMotors.move(outputs.right);
}

void Chassis::curvature(Number throttle, Number turn, bool disableDriveCurve) {
    const Number t = throttle / CONTROLLER_ANALOG_MAX;
    const Number s = turn / CONTROLLER_ANALOG_MAX;
    const Number curvedThrottle = disableDriveCurve ? t : m_throttleCurve(t);
    const Number curvedTurn = disableDriveCurve ? s : m_turnCurve(s);
    const DriveOutputs outputs = desaturate(curvedThrottle, curvedTurn * units::abs(curvedThrottle));
    m_leftMotors.move(outputs.left);
    m_rightMotors.move(outputs.right);
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
