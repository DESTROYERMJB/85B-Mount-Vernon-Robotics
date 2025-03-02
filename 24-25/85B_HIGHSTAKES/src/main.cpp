

#include  "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp"
#include "robodash/api.h"
ASSET(redauton_txt);
ASSET(redauton1_txt);
ASSET(redauton2_txt);
ASSET(redauton3_txt);
ASSET(redauton4_txt);
ASSET(redauton5_txt);
ASSET(blueauton_txt);
ASSET(blueauton1_txt);
ASSET(blueauton2_txt);
ASSET(blueauton3_txt);
ASSET(blueauton4_txt);
ASSET(blueauton5_txt);
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
ASSET(skills12_txt);
ASSET(skills13_txt);
ASSET(skills14_txt);
ASSET(skills15_txt);
ASSET(skills16_txt);
ASSET(skills17_txt);
ASSET(skills18_txt);
ASSET(skills19_txt);
ASSET(skills20_txt);
ASSET(skills21_txt);
ASSET(skills22_txt);
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({-18,-16,-20},pros::MotorGearset::blue);
pros::MotorGroup rightMotors({13,12,15},pros::MotorGearset::blue);
pros::MotorGroup lift({-11,10},pros::MotorGearset::green);
lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              16,
                              lemlib::Omniwheel::NEW_325,
                              360,
                              2);
pros::Motor intake(1,pros::MotorGearset::green);
pros::Optical color_sensor(9);
//odom parts
pros::Imu imu(21);
pros::Rotation horizontalEnc(17);
pros::Rotation verticalEnc(-19);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_325, -0.5);
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
                                              5, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              10 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(5, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              55, // derivative gain (kD)
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
//auton task
//auton variable 0=red 1=blue
int Alliance = 0;
bool match=false;
void red_auton(){
    match=true;
    pros::ADIPneumatics clamp('a',false,false);
    Alliance=0;
    //auton
    chassis.setPose(-53.625, 42.625, 305);
    chassis.follow(redauton_txt,15,3000,false,false);
    clamp.extend();
    intake.move(127);
    pros::delay(500);
    chassis.turnToHeading(0,1000,{},false);
    chassis.follow(redauton1_txt,10,3000,true,false);
    chassis.turnToHeading(90,1000,{},false);
    chassis.follow(redauton2_txt,10,3000,true,false);
    chassis.follow(redauton3_txt,10,3000,false,false);
    //intake.move(0);
    //chassis.turnToHeading(245,1000,{},false);
    //lift.move_absolute(200,200);
    //chassis.follow(redauton4_txt,5,5000,true,false);
    //chassis.turnToHeading(270,1000,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE},false);
    //chassis.follow(redauton5_txt,10,2000,true,false);
    //lift.move_absolute(110,200);
}
void blue_auton(){
    match=true;
    pros::ADIPneumatics clamp('a',false,false);
    Alliance=1;
    //auton
    chassis.setPose(53.625, 42.625, 55);
    chassis.follow(blueauton_txt,15,3000,false,false);
    clamp.extend();
    intake.move(127);
    pros::delay(500);
    chassis.turnToHeading(5,1000,{},false);
    chassis.follow(blueauton1_txt,10,3000,true,false);
    chassis.turnToHeading(270,1000,{},false);
    chassis.follow(blueauton2_txt,10,3000,true,false);
    chassis.follow(blueauton3_txt,10,3000,false,false);
    intake.move(0);
    chassis.turnToHeading(155,1000,{},false);
    lift.move_absolute(200,200);
    chassis.follow(blueauton4_txt,10,3000,true,false);
    chassis.turnToHeading(90,1000,{.direction=AngularDirection::CCW_COUNTERCLOCKWISE},false);
    chassis.follow(blueauton5_txt,10,2000,true,false);
    lift.move_absolute(110,200);
}

// Programming Skills
void skills(){
    pros::ADIPneumatics clamp('a',false,false);
    Alliance=0;
    // Start
    //1
    chassis.setPose(-60.25,0,270); //Origin
    chassis.follow(skills_txt,10,4000,false,false); //Move
    // 2- Grab Clamp
    chassis.turnToHeading(180,1000,{},false); //Rotate
    chassis.follow(skills1_txt,10,4000,false,false); //Move
    clamp.extend(); //Clamp
    pros::delay(200); //Delay 
    intake.move(127); //Start Intake (Forward)
    pros::delay(200); //Delay
    // 3
    chassis.turnToHeading(90,1000,{},false); //Rotate
    chassis.follow(skills2_txt,10,4000,true,false); //Move
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD); //Initalize Lift Motors
    lift.move_absolute(350,200); //Rotate Lift Motors (Top Level)
    pros::delay(200); //Delay
    chassis.turnToHeading(40,1000,{},false); //Rotate
    // 4
    chassis.follow(skills3_txt,6,4000,true,false); //Move
    pros::delay(500); //Delay
    chassis.turnToHeading(0,1000,{},false); //Rotate
    // 5- Wall Stake #1
    chassis.follow(skills4_txt,10,2000,true,false); //Move
    lift.move_absolute(160,200); //Rotate Lift Motors (Middle Level)
    pros::delay(200); //Delay
    // 6- Drop WS Mech
    chassis.follow(skills5_txt,10,2000,false,false); //Move
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); //Release Motors
    lift.brake(); //Release Motors
    chassis.turnToHeading(270,1000,{},false); //Rotate
    // 7
    chassis.follow(skills6_txt,10,10000,true,false); //Move
    // 8
    chassis.follow(skills7_txt,10,5000,false,false); //Move
    chassis.turnToHeading(315,1000,{},false); //Rotate
    // 9
    chassis.follow(skills8_txt,8,2000,true,false); //Move
    pros::delay(500); //Delay
    chassis.turnToHeading(150,2000,{},false); //Rotate
    // 10 - Drop Mobile Goal #1
    chassis.follow(skills9_txt,8,2000,false,false); //Move
    clamp.retract(); //Clamp Up
    intake.move(-127); //Move Intake Backwards
    pros::delay(200); //Delay
    // 11
    chassis.follow(skills10_txt,10,2000,true,false); //Move
    intake.move(127); //Intake Moving Fowards
    chassis.turnToHeading(0,1000,{},false); //Move
    // 12
    chassis.follow(skills11_txt,20,5000,false,false); //Move


    // 13- Start of Quarter 2
    chassis.follow(skills12_txt,10,4000,false,false); //Move
    clamp.extend(); //Clamp
    pros::delay(200); //Delay 
    intake.move(127); //Start Intake (Forward)
    pros::delay(200); //Delay
    // 3
    chassis.turnToHeading(180,1000,{},false); //Rotate
    chassis.follow(skills13_txt,10,4000,true,false); //Move
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD); //Initalize Lift Motors
    lift.move_absolute(350,200); //Rotate Lift Motors (Top Level)
    pros::delay(200); //Delay
    chassis.turnToHeading(140,1000,{},false); //Rotate
    // 4
    chassis.follow(skills14_txt,6,4000,true,false); //Move
    pros::delay(500); //Delay
    chassis.turnToHeading(180,1000,{},false); //Rotate
    // 5- Wall Stake #1
    chassis.follow(skills15_txt,10,2000,true,false); //Move
    lift.move_absolute(160,200); //Rotate Lift Motors (Middle Level)
    pros::delay(200); //Delay
    // 6- Drop WS Mech
    chassis.follow(skills16_txt,10,2000,false,false); //Move
    lift.set_brake_mode(pros::E_MOTOR_BRAKE_COAST); //Release Motors
    lift.brake(); //Release Motors
    chassis.turnToHeading(270,1000,{},false); //Rotate
    // 7
    chassis.follow(skills17_txt,10,10000,true,false); //Move
    // 8
    chassis.follow(skills18_txt,10,5000,false,false); //Move
    chassis.turnToHeading(225,1000,{},false); //Rotate
    // 9
    chassis.follow(skills19_txt,8,2000,true,false); //Move
    pros::delay(500); //Delay
    chassis.turnToHeading(210,2000,{},false); //Rotate
    // 10 - Drop Mobile Goal #1
    chassis.follow(skills20_txt,8,2000,false,false); //Move
    clamp.retract(); //Clamp Up
    intake.move(-127); //Move Intake Backwards
    pros::delay(200); //Delay
    // 11
    chassis.follow(skills21_txt,10,2000,true,false); //Move
    intake.move(127); //Intake Moving Fowards
    chassis.turnToHeading(90,1000,{},false); //Move
    // 12
    chassis.follow(skills22_txt,20,5000,false,false); //Move
    
}

rd::Selector selector({
    {"Red auto", red_auton},
    {"Blue auto", blue_auton},
    {"Skills", skills}
});
//auton variable 0=red 1=blue
//auton selector
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
rd::Console console;
void initialize() {
    pros::Controller master(pros::E_CONTROLLER_MASTER);
    chassis.calibrate(); // calibrate sensors
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    lift.set_encoder_units_all(pros::E_MOTOR_ENCODER_DEGREES);
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
 * for non-competition testing purposes. shellyisastickycummonster
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
    selector.run_auton();
}


//Task for scoring rings
int clampState = 1;
int rolling = 0;

void redirect(){
    intake.move(80);
    if(Alliance==0){
        while(color_sensor.get_hue()>30){
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)==1){
                intake.move(127);
                rolling=1;
                return;
            }
            pros::delay(15);
        }
        intake.move_relative(50,50);
        pros::delay(200);
        intake.move_relative(-500,50);
    }else{
        while(color_sensor.get_hue()>230 and color_sensor.get_hue()<200){
            if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)==1){
                return;
                intake.move(127);
                rolling=1;
            }
            pros::delay(15);
        }
        intake.move_relative(50,50);
        pros::delay(200);
        intake.move_relative(-500,50);
    }
    pros::delay(3000);
    intake.move(127);
}

void ring(){
    int rolling=0;
    intake.set_encoder_units(pros::E_MOTOR_ENCODER_DEGREES);
    while(true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)==1 and rolling==0){
            intake.move(127);
            rolling=1;
            pros::delay(200);
        }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)==1 and rolling==0){
            intake.move(-127);
            rolling=1;
            pros::delay(200);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)==1 and rolling ==1){
            intake.move(0);
            rolling=0;
            pros::delay(200);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)==1){
            redirect();
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)==1){
            lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
            lift.move_absolute(350,200);
            pros::delay(200);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)==1){
            lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
            lift.brake();
            pros::delay(200);
        } else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)==1){
            lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
            lift.move_absolute(200,90);
            pros::delay(200);
        } else{
            pros::delay(20);
        }
    }
}


//task that runs the clamp
void ClampF(){
    pros::ADIPneumatics clamp('a',false,false);
    if(match){
        clamp.extend();
    }
    while(true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) and clampState==0){
            clamp.extend();
            clampState=1;
            pros::delay(500);
        } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2) and clampState==1){
            clamp.retract();
            clampState=0;
            pros::delay(500);
        } else{
            pros::delay(20);
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
    color_sensor.set_led_pwm(100);
    chassis.cancelAllMotions();
	pros::Controller master(pros::E_CONTROLLER_MASTER);
    pros::Task ringTask(ring);
    pros::Task clampTask(ClampF);
    color_sensor.set_led_pwm(100);
    //color_sensor.disable_gesture();
	while (true) {
		int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightY = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(leftY,rightY);
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)==1){
            lift.set_zero_position_all(0);
        }
        console.clear();
        //if(color_sensor.get_hue()<=20 and color_sensor.get_hue()>=0){
        //    console.print("Red Ring");
        //} else if(color_sensor.get_hue()<=230 and color_sensor.get_hue()>=200){
        //    console.print("Blue Ring");
        //}
        console.printf("prox: %d",color_sensor.get_proximity());
        pros::delay(25);
	}
}
