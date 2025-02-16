

#include  "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
ASSET(skills_txt);
ASSET(skills1_txt);
ASSET(skills2_txt);
ASSET(skills3_txt);
ASSET(skills4_txt);
ASSET(skills5_txt);
ASSET(skills6_txt);
ASSET(skills7_txt);
ASSET(skills8_txt);
ASSET(skills9_txt);
ASSET(skills10_txt);
ASSET(skills11_txt);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({-15,-19,-17},pros::MotorGearset::blue);
pros::MotorGroup rightMotors({9,10,8},pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              16,
                              lemlib::Omniwheel::NEW_325,
                              360,
                              2);
pros::Motor intake(1,pros::MotorGearset::green);
pros::Motor conveyor(-12,pros::MotorGearset::red);
//odom parts
pros::Imu imu(13);
pros::Rotation horizontalEnc(2);
pros::Rotation verticalEnc(14);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, -0.125);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_325, -1.625);
//odometry setting
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set      to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(15, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              7, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(7, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              60, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
int clampState = 1;
int rolling = 0;

void ring(){
    int rolling=0;
    while(true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)==1 and rolling==0){
            intake.move(127);
            conveyor.move(127);
            rolling=1;
            pros::delay(500);
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)==1 and rolling==0){
            intake.move(-127);
            conveyor.move(-90);
            rolling=1;
            pros::delay(500);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)==1 and rolling ==1){
            intake.move(0);
            conveyor.move(0);
            rolling=0;
            pros::delay(500);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)==1 and rolling ==1){
            intake.move(-127);
            conveyor.move(-90);
            pros::delay(500);
            intake.move(127);
            conveyor.move(127);
        }else {
            pros::delay(25);
        }
    }
}

//auton selector
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    chassis.calibrate(); // calibrate sensors
    pros::lcd::initialize();
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    conveyor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    pros::delay(25);
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
void competition_initialize() {
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    pros::ADIPneumatics clamp('a',false,false);
    chassis.setPose(-58.092,17.826,240);
    chassis.follow(skills_txt,10,2000,false,false);
    clamp.extend();
    pros::delay(200);
    intake.move(127);
    conveyor.move(127);
    pros::delay(200);
    chassis.turnToHeading(90,1000,{},false);
    chassis.follow(skills1_txt,5,2000,true,false);
    chassis.turnToHeading(0,1000,{},false);
    chassis.follow(skills2_txt,5,10000,true,false);
    pros::delay(500);
    chassis.follow(skills3_txt,10,2000,false,false);
    chassis.follow(skills4_txt,10,2000,true,false);
    chassis.turnToHeading(150,1000,{},false);
    chassis.follow(skills5_txt,10,2000,false,false);
    clamp.retract();

}
void ClampF(){
    pros::ADIPneumatics clamp('a',false,false);
    while(true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) and clampState==0){
            clamp.extend();
            clampState=1;
            pros::delay(200);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) and clampState==1){
            clamp.retract();
            clampState=0;
            pros::delay(200);
        } else{
            pros::delay(25);
        }
    }
}
/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *z
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
    chassis.cancelAllMotions();
    pros::lcd::set_text(2, "Driver Control");
	pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Task ringTask(ring);
    pros::Task clampTask(ClampF);
	while (true) {
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY,rightY);
        pros::delay(25);
	}
}
