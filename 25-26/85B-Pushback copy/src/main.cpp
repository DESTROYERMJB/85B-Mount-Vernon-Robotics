#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/rtos.hpp"

asset(skills1copy_txt);

pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Optical upper(11);
pros::MotorGroup RD({-10,9,8},pros::MotorGearset::blue);
pros::MotorGroup LD({20,-19,-18},pros::MotorGearset::blue);
pros::Motor Lintake(2,pros::MotorGearset::blue);
pros::Motor Uintake(-1,pros::MotorGearset::green);
pros::Motor Outake(-3,pros::MotorGearset::green);

lemlib::Drivetrain drivetrain(&LD, // left motor group
                              &RD, // right motor group
                              10.25, // 10 inch track width
                              lemlib::Omniwheel::NEW_275,
                              450, // drivetrain rpm is 360
                              2 // horizontal drift is 2 (for now)
);
// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              32, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acc. eleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(3, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              27, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve1(0, // joystick deadband out of 127
                                     30, // minimum output where drivetrain will move out of 127
                                     1.06 // expo curve gain
);

pros::Imu imu(4);
// horizontal tracking wheel encoder
// vertical tracking wheel encoder
pros::Rotation vertical_encoder(-7);
// horizontal tracking wheel
// vertical tracking wheel
lemlib::TrackingWheel vertical_tracking_wheel(&vertical_encoder, lemlib::Omniwheel::NEW_275, 0.25);

// odometry settings
lemlib::OdomSensors sensors(&vertical_tracking_wheel, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            nullptr,//&horizontal_tracking_wheel, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);
// create the chassis
lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors, // odometry sensors
                        &throttle_curve1,
                        &throttle_curve1
);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */

void initialize() {
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    chassis.calibrate();
    Lintake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Uintake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    Outake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void skills() {
    //startup
    pros::adi::Pneumatics Scraper('a',false);
    pros::adi::Pneumatics Flap('b',false);
    Lintake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    Uintake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    Outake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    upper.set_led_pwm(100);
    upper.disable_gesture();
    chassis.setPose(-48.75,-7.5,180);

    // start of auton
    chassis.setPose(-48.75,-7.5,180);
    chassis.moveToPose(-48.75, -48, 180, 4000,{},false);
    chassis.turnToHeading(270, 2000,{},false);
    Scraper.extend();
    Lintake.move(127);
    Uintake.move(127);
    Outake.move(127);
    pros::delay(650);
    chassis.moveToPose(-62, -48, 270, 2000,{.maxSpeed=70},false);
    pros::delay(4000);

    // retracts from loader
    chassis.moveToPose(-48, -48,270, 2000,{.forwards=false},false);
    Scraper.retract();
    chassis.turnToHeading(150, 1000);
    chassis.moveToPose(-24,-60,90,2000,{},false);
    chassis.turnToHeading(90, 1000, {} , false);
    chassis.moveToPose(24, -60, 90, 3000, {}, false);
    chassis.turnToHeading(60, 1000);
    chassis.moveToPoint(48, -47.5, 3000);
    chassis.turnToHeading(90, 1000);
    chassis.moveToPose(20, -47.5, 90, 2000, {.forwards=false}, false);
    chassis.setPose(31.5,-47,chassis.getPose().theta,false);
    Flap.extend();
    pros::delay(1950);
    chassis.moveToPose(48, -47,90, 2000,{},false);
    Flap.retract();
    Scraper.extend();
    pros::delay(750);
    chassis.moveToPose(62, -47, 90, 2000,{.maxSpeed=70},false);
    pros::delay(4000);
    chassis.moveToPose(31.5, -47, 90, 2000, {.forwards=false}, true);
    Scraper.retract();
    chassis.waitUntilDone();
    Flap.extend();
    pros::delay(1950);
    chassis.moveToPoint(48, -47, 1000, {.minSpeed=50,.earlyExitRange=5},false);
    Flap.retract();
    chassis.moveToPoint(36, 24, 3000, {.minSpeed=50,.earlyExitRange=8},false);
    chassis.moveToPoint(48, 48, 2000,{},false);
    chassis.turnToHeading(90, 1000,{},false);
    Scraper.extend();
    pros::delay(650);
    chassis.moveToPose(62, 48, 90, 2000,{.maxSpeed=70},false);
    pros::delay(4000);
    chassis.moveToPose(48, 48,90, 2000,{.forwards=false},false);
}

void RightAuton(){
    //startup
    pros::adi::Pneumatics Scraper('a',false);
    pros::adi::Pneumatics Flap('b',false);
    Lintake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    Uintake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    Outake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    upper.set_led_pwm(100);
    upper.disable_gesture();
    chassis.setPose(-48.5,-7.5,180);


    chassis.moveToPose(-48.5, -48, 180, 4000,{},false);
    chassis.turnToHeading(270, 2000,{},false);
    Scraper.extend();
    Lintake.move(127);
    Uintake.move(127);
    Outake.move(127);
    pros::delay(650);
    chassis.moveToPose(-62, -48, 270, 2000,{.minSpeed=100},false);
    pros::delay(4000);
    chassis.moveToPose(-30, -48,270, 3000,{.forwards=false});
    pros::delay(250);
    Scraper.retract();
    chassis.waitUntilDone();
    Flap.extend();
    pros::delay(500);
    Uintake.move(-127);
    Lintake.move(-127);
    pros::delay(500);
    Uintake.move(127);
    pros::delay(850);
    Uintake.move(-127);
    RD.move_relative(540, 100);
    LD.move_relative(540, 100);
    pros::delay(500);
    Flap.retract();
    RD.move_relative(-600, 600);
    LD.move_relative(-600, 600);
    //hola yo soy dora 
}

void LeftAuton(){
    pros::adi::Pneumatics Scraper('a',false);
    pros::adi::Pneumatics Flap('b',false);
    Lintake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    Uintake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    Outake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    upper.set_led_pwm(100);
    upper.disable_gesture();
    chassis.setPose(48.5,-7.5,180);


    chassis.moveToPose(48.5, -48, 180, 4000,{},false);
    chassis.turnToHeading(90, 2000,{},false);
    Scraper.extend();
    Lintake.move(127);
    Uintake.move(127);
    Outake.move(127);
    pros::delay(650);
    chassis.moveToPose(62, -48, 90, 2000,{.minSpeed=127},false);
    pros::delay(4000);
    chassis.moveToPose(30, -48,90, 3000,{.forwards=false});
    pros::delay(250);
    Scraper.retract();
    chassis.waitUntilDone();
    Flap.extend();
    pros::delay(500);
    Uintake.move(-127);
    Lintake.move(-127);
    pros::delay(500);
    Uintake.move(127);
    pros::delay(800);
    Uintake.move(-127);
    RD.move_relative(540, 100);
    LD.move_relative(540, 100);
    pros::delay(500);
    Flap.retract();
    RD.move_relative(-600, 600);
    LD.move_relative(-600, 600);
}

void autonomous() {
    skills();
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *1=]2
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
 void loader(){
    bool load = false;
    bool flap = false;
    pros::adi::Pneumatics Elevation('a',false);
    pros::adi::Pneumatics Flap('b',false);
    while(true){
        //loader
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && !load){
			Elevation.extend();
            Flap.retract();
            flap=false;
            load = true;
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && load) {
            Elevation.retract();
            load = false;
        }
        //flap
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && !flap){
			Elevation.retract();
            Flap.extend();
            flap = true;
            load=false;
		} else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) && flap) {
            Flap.retract();
            flap = false;
        }
        pros::delay(200);
    }
 }
void opcontrol() {
    pros::Task::create(loader);
	chassis.setBrakeMode(pros::E_MOTOR_BRAKE_COAST);
	while (true) {
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            Lintake.move(127);
            Uintake.move(127);
            Outake.move(127);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
			Lintake.move(127);
            Uintake.move(127);
            Outake.move(-127);
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            Lintake.move(-127);
            Uintake.move(-127);
            Outake.move(127);
        } else{
            Lintake.move(0);
            Uintake.move(0);
            Outake.move(0);
        }
		// get left y and right y positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        
        // move the robot
        chassis.tank(leftY, rightY);
		pros::delay(20);                               // Run for 20 ms then update
	}
}