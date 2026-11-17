#include "robot.hpp"
#include "hardware/IMU/V5InertialSensor.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"

// Plain numbers below are in the standard units defined in UnitConfig.hpp (inches, degrees, milliseconds, rpm).
// TODO: confirm ports, cartridges, offsets, wheel sizes, and measurements, then tune the controllers

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup intake({10,11}, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);
pros::MotorGroup lift({-14, 15}, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);

lemlib::Drivetrain drivetrain(
    lemlib::MotorGroup({1, 2, 3}, lemlib::Cartridge::BLUE),
    lemlib::MotorGroup({4, 5, 6}, lemlib::Cartridge::BLUE),
    13.875,
    3.25,
    450,
    5
);

lemlib::LateralController lateralController(1, 0, 0, 0, 1, 200, 3, 500, 0);
lemlib::AngularController angularController(0.05, 0, 0, 0, 1, 2000, 3, 500, 0);

// odom sensors. Rotation sensors on smart ports (negative port = reversed)
// TODO: confirm ports, reversals, wheel diameters, and offsets
lemlib::V5InertialSensor imu(1);
lemlib::TrackingWheel verticalTracker(2, 2.75_in, 26.5_cm / 2);
lemlib::TrackingWheel horizontalTracker(-3, 2.75_in, -26.5_cm / 2);
lemlib::OdomSensors odomSensors(verticalTracker, horizontalTracker, imu);

lemlib::Chassis chassis(drivetrain, lateralController, angularController, odomSensors, lemlib::DriveType::TANK);
