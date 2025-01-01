#include  "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
ASSET(skills_txt);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({-15,-19,-17},pros::MotorGearset::blue);
pros::MotorGroup rightMotors({9,10,8},pros::MotorGearset::blue);
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              12,
                              lemlib::Omniwheel::NEW_325,
                              360,
                              2);
pros::Motor intake(1,pros::MotorGearset::green);
pros::Motor conveyor(-12,pros::MotorGearset::red);
//odom parts
pros::Imu imu(13);
pros::Rotation horizontalEnc(-3);
pros::Rotation verticalEnc(-4);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, 0);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_325, 0);
//odometry setting
lemlib::OdomSensors sensors(&vertical, // vertical tracking wheel 1, set to null
                            nullptr, // vertical tracking wheel 2, set to nullptr as we are using IMEs
                            &horizontal, // horizontal tracking wheel 1
                            nullptr, // horizontal tracking wheel 2, set      to nullptr as we don't have a second one
                            &imu // inertial sensor
);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in inches
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in inches
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              0, // anti windup
                                              0, // small error range, in degrees
                                              0, // small error range timeout, in milliseconds
                                              0, // large error range, in degrees
                                              0, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(drivetrain, // drivetrain settings
                        lateral_controller, // lateral PID settings
                        angular_controller, // angular PID settings
                        sensors // odometry sensors
);
int clampState = 0;
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
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)==1 or controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)==1 and rolling ==1){
            intake.move(0);
            conveyor.move(0);
            rolling=0;
            pros::delay(500);
        }
    }
}


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
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    pros::ADIPneumatics clamp('a',false,false);
    chassis.setPose(0, 0, 0);
    // turn to face heading 90 with a very long timeout
    //chassis.turnToHeading(90, 100000);
    intake.move(127);
    conveyor.move(127);
    leftMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
    rightMotors.set_encoder_units_all(MOTOR_ENCODER_DEGREES);
    pros::delay(500);
    leftMotors.move_relative(1080,50);
    rightMotors.move_relative(1080,50);
    while(not imu.get_heading()<=270){
        leftMotors.move(-40);
        rightMotors.move(40);
    }
    leftMotors.brake();
    rightMotors.brake();
    leftMotors.move_relative(-1080,50);
    rightMotors.move_relative(-1080,50);
    clamp.extend();
    while(not imu.get_heading()>=90){
        leftMotors.move(40);
        rightMotors.move(-40);
    }
    leftMotors.brake();
    rightMotors.brake();

}
void ClampF(){
    pros::ADIPneumatics clamp('a',false,false);
    while(true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) and clampState==0){
            clamp.extend();
            clampState=1;
            pros::delay(500);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) and clampState==1){
            clamp.retract();
            clampState=0;
            pros::delay(500);
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
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
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
