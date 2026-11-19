#include "robot.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/optical.hpp"

// Plain numbers below are in the standard units defined in UnitConfig.hpp (inches, degrees, milliseconds, rpm).
// TODO: confirm ports, cartridges, offsets, wheel sizes, and measurements, then tune the controllers

pros::Controller driveController(pros::E_CONTROLLER_MASTER);
pros::Controller liftController(pros::E_CONTROLLER_PARTNER);

pros::adi::Pneumatics blockerPiston('B', false);
pros::Motor intake(19, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::adi::Pneumatics intakePiston('A', false);
pros::MotorGroup lift({-11, 20}, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);
pros::adi::Pneumatics wrist('C', false);
pros::adi::Pneumatics claw('D', false);
pros::Distance leftDistance(5);
pros::Distance rightDistance(6);
pros::Optical leftColor(13);
pros::Optical rightColor(7);

lemlib::Drivetrain drivetrain(
    lemlib::MotorGroup({-1, -2, -3}, lemlib::Cartridge::BLUE),
    lemlib::MotorGroup({10, 9, 8}, lemlib::Cartridge::BLUE),
    13.875,
    3.25,
    450,
    5
);

lemlib::LateralController lateralController(1, 0, 0, 0, 1, 200, 3, 500, 0);
lemlib::AngularController angularController(0.05, 0, 0, 0, 1, 2000, 3, 500, 0);

// odom sensors. Rotation sensors on smart ports (negative port = reversed)
// TODO: confirm ports, reversals, wheel diameters, and offsets
lemlib::V5InertialSensor imu(18);
lemlib::TrackingWheel verticalTracker(-17, 2.75_in, 26.5_cm / 2);
lemlib::TrackingWheel horizontalTracker(14, 2.75_in, -26.5_cm / 2);
lemlib::OdomSensors odomSensors(verticalTracker, horizontalTracker, imu);

lemlib::Chassis chassis(drivetrain, lateralController, angularController, odomSensors, lemlib::DriveType::TANK);
