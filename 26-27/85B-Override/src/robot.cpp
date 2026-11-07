#include "robot.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "pros/abstract_motor.hpp"

// Plain numbers below are in the standard units defined in UnitConfig.hpp (inches, degrees, milliseconds, rpm).
// TODO: confirm ports, cartridges, offsets, wheel sizes, and measurements, then tune the controllers

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup lift({14, -15}, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);

lemlib::Drivetrain Drivetrain(
    lemlib::MotorGroup({1, 2, 3}, lemlib::Cartridge::BLUE),
    lemlib::MotorGroup({4, 5, 6}, lemlib::Cartridge::BLUE),
    13.875,
    3.25,
    450,
    5
);

lemlib::LateralController LateralController(1, 0, 0, 0, 1, 200, 3, 500, 0);
lemlib::AngularController AngularController(0.05, 0, 0, 0, 1, 2000, 3, 500, 0);

// odom sensors. Rotation sensors on smart ports (negative port = reversed)
// TODO: confirm ports, reversals, wheel diameters, and offsets
lemlib::V5InertialSensor imu(1);
lemlib::TrackingWheel verticalTracker(2, 2.75_in, 26.5_cm / 2);
lemlib::TrackingWheel horizontalTracker(-3, 2.75_in, -26.5_cm / 2);
lemlib::OdomSensors OdomSensors(verticalTracker, horizontalTracker, imu);

lemlib::Chassis chassis(Drivetrain, LateralController, AngularController, OdomSensors, lemlib::DriveType::TANK);
