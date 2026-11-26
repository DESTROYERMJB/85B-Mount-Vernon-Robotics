#include "main.h"
#include "hardware/Motor/Motor.hpp"
#include "pros/misc.h"
#include "robot.hpp"

// Can mess with bot
// MODIFY CAREFULLY OTHERWISE CONSULT BRADBURY
//Control Scheme: False = Match (Default), True = testing
bool Testing = false;

// CAN MESS WITH BOT
// MODIFY CAREFULLY OTHERWISE CONSULT BRADBURY
// -1 = unselected
// Alliance: 0 = Red, 1 = Blue
// Routine:  0 = Left, 1 = Right, 2 = Skills (Red only)
int selected_alliance = -1;
int selected_routine = -1;

// CAN BREAK BOT
// DO NOT TOUCH WITHOUT CONSULTING BRADBURY 
// UI object pointers needed inside callbacks
lv_obj_t * testing_btn = nullptr;
lv_obj_t * alliance_btnm = nullptr;
lv_obj_t * routine_btnm = nullptr;

// CAN BREAK BOT
// DO NOT TOUCH WITHOUT CONSULTING BRADBURY
// Global pointers for your telemetry text labels
lv_obj_t * x_label;
lv_obj_t * y_label;
lv_obj_t * heading_label;
lv_obj_t * task_counter_label;

// DO NOT TOUCH WITHOUT CONSULTING BRADBURY
// Button maps for LVGL 9 button matrices (always end with "")
static const char * testing_map[] = {
    "Match", "Testing", ""
};

// DO NOT TOUCH WITHOUT CONSULTING BRADBURY
static const char * alliance_map[] = {
    "Red", "Blue", ""
};

// DO NOT TOUCH WITHOUT CONSULTING BRADBURY
static const char * red_routine_map[] = {
    "Left", "Right", "\n",
    "Skills", ""
};

// DO NOT TOUCH WITHOUT CONSULTING BRADBURY
static const char * blue_routine_map[] = {
    "Left", "Right", ""
};

// DO NOT TOUCH WITHOUT CONSULTING BRADBURY
// Helper to clear the checked highlight on all buttons of a buttonmatrix
static void clear_btnm_selection(lv_obj_t * btnm, uint32_t btn_count) {
    for (uint32_t i = 0; i < btn_count; i++) {
        lv_buttonmatrix_clear_button_ctrl(btnm, i, LV_BUTTONMATRIX_CTRL_CHECKED);
    }
}

// CAN BREAK BOT
// DO NOT TOUCH WITHOUT CONSULTING BRADBURY
// Callback for Testing switch ("Match" or "Testing")
void testing_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;

    lv_obj_t * obj = (lv_obj_t *)lv_event_get_target(e);
    uint32_t id = lv_buttonmatrix_get_selected_button(obj);
    if (id == LV_BUTTONMATRIX_BUTTON_NONE) return;

    const char * txt = lv_buttonmatrix_get_button_text(obj, id);
    if (!txt) return;

    if (strcmp(txt, "Match") == 0) {
        Testing = false;
    } else if (strcmp(txt, "Testing") == 0) {
        Testing = true;
    }
}

// CAN BREAK BOT
// DO NOT TOUCH WITHOUT CONSULTING BRADBURY
// Callback for Alliance switch ("Red" or "Blue")
void alliance_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;

    lv_obj_t * obj = (lv_obj_t *)lv_event_get_target(e);
    uint32_t id = lv_buttonmatrix_get_selected_button(obj);
    if (id == LV_BUTTONMATRIX_BUTTON_NONE) return;

    const char * txt = lv_buttonmatrix_get_button_text(obj, id);
    if (!txt) return;

    if (strcmp(txt, "Red") == 0) {
        selected_alliance = 0;
        lv_buttonmatrix_set_map(routine_btnm, red_routine_map);
        lv_buttonmatrix_set_button_ctrl_all(routine_btnm, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    } else if (strcmp(txt, "Blue") == 0) {
        selected_alliance = 1;
        lv_buttonmatrix_set_map(routine_btnm, blue_routine_map);
        lv_buttonmatrix_set_button_ctrl_all(routine_btnm, LV_BUTTONMATRIX_CTRL_CHECKABLE);

        // If "Skills" was selected while on Red, clear it since Blue has no Skills here
        if (selected_routine == 2) {
            selected_routine = -1;
            clear_btnm_selection(routine_btnm, 2);
        }
    }
}

// CAN BREAK BOT
// DO NOT TOUCH WITHOUT CONSULTING BRADBURY
// Callback for Routine switch ("Left", "Right", "Skills")
void routine_event_cb(lv_event_t * e) {
    if (lv_event_get_code(e) != LV_EVENT_VALUE_CHANGED) return;

    lv_obj_t * obj = (lv_obj_t *)lv_event_get_target(e);
    uint32_t id = lv_buttonmatrix_get_selected_button(obj);
    if (id == LV_BUTTONMATRIX_BUTTON_NONE) return;

    const char * txt = lv_buttonmatrix_get_button_text(obj, id);
    if (!txt) return;

    if (strcmp(txt, "Left") == 0) {
        selected_routine = 0;
    } else if (strcmp(txt, "Right") == 0) {
        selected_routine = 1;
    } else if (strcmp(txt, "Skills") == 0) {
        if (selected_alliance == 0) {
            selected_routine = 2;
        } else {
            selected_routine = -1;
        }
    }
}

// CAN BREAK BOT
// DO NOT TOUCH WITHOUT CONSULTING BRADBURY
// Setup the LVGL screen with tabs and button matrices
void screenSetup() {
    lv_obj_t * tabview = lv_tabview_create(lv_screen_active());
    lv_obj_t * auton_tab = lv_tabview_add_tab(tabview, "Auton Selector");
    lv_obj_t * developer_tab = lv_tabview_add_tab(tabview, "Developer");
    // 1. Testing switch (top of Developer tab)
    lv_obj_t * testing_btn = lv_buttonmatrix_create(developer_tab);
    lv_buttonmatrix_set_map(testing_btn, testing_map);
    lv_obj_set_size(testing_btn, 420, 50);
    lv_obj_align(testing_btn, LV_ALIGN_TOP_MID, 0, 4);
    lv_buttonmatrix_set_button_ctrl_all(testing_btn, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_one_checked(testing_btn, true);

    // 2. Alliance switch (top of Auton tab)
    alliance_btnm = lv_buttonmatrix_create(auton_tab);
    lv_buttonmatrix_set_map(alliance_btnm, alliance_map);
    lv_obj_set_size(alliance_btnm, 420, 50);
    lv_obj_align(alliance_btnm, LV_ALIGN_TOP_MID, 0, 8);
    lv_buttonmatrix_set_button_ctrl_all(alliance_btnm, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_one_checked(alliance_btnm, true);
    lv_obj_add_event_cb(alliance_btnm, alliance_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // 3. Routine switch (below Alliance switch; defaults to Red's map until tapped)
    routine_btnm = lv_buttonmatrix_create(auton_tab);
    lv_buttonmatrix_set_map(routine_btnm, red_routine_map);
    lv_obj_set_size(routine_btnm, 420, 115);
    lv_obj_align(routine_btnm, LV_ALIGN_TOP_MID, 0, 68);
    lv_buttonmatrix_set_button_ctrl_all(routine_btnm, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_one_checked(routine_btnm, true);
    lv_obj_add_event_cb(routine_btnm, routine_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);


    // Developer tab labels (keep your existing setup here)
    x_label = lv_label_create(developer_tab);
    lv_label_set_text(x_label, "X: 0.00");
    lv_obj_align(x_label, LV_ALIGN_TOP_LEFT, 20, 60);

    y_label = lv_label_create(developer_tab);
    lv_label_set_text(y_label, "Y: 0.00");
    lv_obj_align(y_label, LV_ALIGN_TOP_LEFT, 20, 80);

    heading_label = lv_label_create(developer_tab);
    lv_label_set_text(heading_label, "Heading: 0.00 deg");
    lv_obj_align(heading_label, LV_ALIGN_TOP_LEFT, 20, 100);

    task_counter_label = lv_label_create(developer_tab);
    lv_label_set_text(task_counter_label, "Task Counter: 0");
    lv_obj_align(task_counter_label, LV_ALIGN_TOP_LEFT, 20, 120);
}

// CAN BREAK BOT
// YOU CAN CARFELY MODIFY BUT DON'T DELETE ANYTHING IN INITIALIZE WITHOUT CONSULTING BRADBURY
// Main initialization function called when the robot starts up
void initialize() {

    // Initialize the LVGL library and create the screen
    screenSetup();

    // Initialize the robot hardware and subsystems
    rightColor.set_led_pwm(100);
    leftColor.set_led_pwm(100);
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    lift.set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
    lift.tare_position_all();

    // Calibrates Odom
    chassis.calibrate();

    // Rumble the controllers to indicate that initialization is complete
    driveController.rumble("-..-");
    liftController.rumble("-..-");
}

void disabled() {}

void autonomous() {
    chassis.setBrakeMode(lemlib::BrakeMode::HOLD);
    
    while (selected_alliance == -1 && selected_routine == -1) {
        pros::delay(50);
    }

    // Disallow Skills if Blue is selected
    if (selected_alliance == 1 && selected_routine == 2) {
        return;
    }

    if (selected_alliance == 0) {
        // RED ALLIANCE
        switch (selected_routine) {
            case 0:
                // Red Left
                break;
            case 1:
                // Red Right
                break;
            case 2:
                // Red Skills
                break;
            default:
                break;
        }
    } else if (selected_alliance == 1) {
        // BLUE ALLIANCE
        switch (selected_routine) {
            case 0:
                // Blue Left
                break;
            case 1:
                // Blue Right
                break;
            default:
                break;
        }
    }
}

// Global variables for whether the intake is in match load mode
bool matchLoad = false;

// Global variables for whether the blocking arm is down
bool blocking = false;

// Global variables for intake motor control
// spinForToggle indicates the intake is spinning to flip the toggle
bool spinForToggle = false;
// spinForIntake indicates the intake is spinning to intake or outtake
bool spinForIntake = false;
// spinReverse indicates the intake is spinning in reverse (outtake)
bool spinReverse = false;
// Global variable for whether we are spinning the toggle for the right side (true) or left side (false)
bool toggleRight = false;

// Function to check if the color detected by the optical sensor matches the selected alliance
bool toggleColorCheck (double hue) {
    if (selected_alliance == 0 && ((hue < 25 || hue > 335))) {
        return true;
    } else if (selected_alliance == 1 && (hue > 200 && hue < 250)) {
        return true;
    } else {
        return false;
    }
}

// Lifetime function to handle the intake motor based on the global variables
void intakeMotorHandler() {
    while (true) {
        if (spinForToggle) {
            while (spinForToggle) {
                if (toggleRight && !toggleColorCheck(rightColor.get_hue())) {
                    intake.move(127);
                } else if (!toggleRight && !toggleColorCheck(leftColor.get_hue())) {
                    intake.move(127);
                } else {
                    intake.move(0);
                }
                pros::delay(10);
            }
        } else if (spinForIntake && !spinReverse) {
            intake.move(127);
        } else if (spinForIntake && spinReverse) {
            intake.move(-127);
        } else {
            intake.move(0);
        }
        pros::delay(10);
    }
}

// Lifetime function to handle the interactions with the intake and blocking arm based on the controller inputs and to give direction to the intake motor handler
void interactions() {
    while (true) {
        if (driveController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && !matchLoad) {
            if(blocking) {
                blockerPiston.retract();
                blocking = false;
                pros::delay(50);
            }
            intakePiston.extend();
            matchLoad = true;
        } else if (driveController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && matchLoad) {
            intakePiston.retract();
            matchLoad = false;
        } else if (driveController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) && !blocking) {
            blockerPiston.extend();
            blocking = true;
        } else if (driveController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT) && blocking) {
            blockerPiston.retract();
            blocking = false;
        } else if (driveController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            spinForToggle = !spinForToggle;
            toggleRight = false;
        } else if (driveController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            spinForToggle = !spinForToggle;
            toggleRight = true;
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            spinForIntake = true;
            spinReverse = false;
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            spinForIntake = true;
            spinReverse = true;
        } else {
            spinForIntake = false;
            spinReverse = false;
        }
        pros::delay(10);
    }
}


// Global variables for wrist and claw states
bool wristExtended = false;
bool clawClamped = false;

// Lifetime function to handle the placement of the wrist and claw based on the controller inputs
void placement() {
    while (true) {
        if (driveController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) && !wristExtended) {
            wrist.extend();
            wristExtended = true;
        } else if (driveController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X) && wristExtended) {
            wrist.retract();
            wristExtended = false;
        }
        if (driveController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) && !clawClamped) {
            claw.extend();
            clawClamped = true;
        } else if (driveController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A) && clawClamped) {
            claw.retract();
            clawClamped = false;
        }

        pros::delay(10);
    }
}

// Global variable for the target level of the lift (0 = bottom, increments of 1 for just a pin, 2 for a pin and a cone,)
int targetLevel = 0;

// Future: Lifetime function to handle the lift based on the controller inputs using pid to controll the lift motors to the target level
void liftHandler() {

}

// DO NOT TOUCH WITHOUT CONSULTING BRADBURY
// Lifetime function to update the telemetry UI with the robot's position and heading
void update_telemetry_ui() {
    while (true) {
        // lv_label_set_text_fmt allows you to format strings exactly like printf
        lv_label_set_text_fmt(x_label, "X: %.2f in", chassis.getX());
        lv_label_set_text_fmt(y_label, "Y: %.2f in", chassis.getY());
        lv_label_set_text_fmt(heading_label, "Heading: %.2f deg", chassis.getHeading());
        lv_label_set_text_fmt(task_counter_label, "Task Counter: %d", pros::Task::get_count());
        pros::delay(100);
    }
}

// Main operator control loop
void opcontrol() {
    // Set the chassis brake mode to COAST for smoother driving during operator control
    chassis.setBrakeMode(lemlib::BrakeMode::COAST);

    // Start the lifetime tasks for interactions, intake motor control, placement, lift control, and telemetry UI updates
    static pros::Task InteractionsTask(interactions);
    static pros::Task intakeMotorTask(intakeMotorHandler);
    static pros::Task placementTask(placement);
    static pros::Task liftTask(liftHandler);
    static pros::Task telemetryTask(update_telemetry_ui);

    // Main operator control loop
    while (true) {
   
        //Current Implementation of lift control, will be replaced with PID control in the future
        if (liftController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.move(127);
        } else if (liftController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            lift.move(-127);
        } else {
            lift.move(0);
        }

        // Pass the drive controller inputs to the chassis for driving control
        chassis.driverControl(driveController);

        // DO NOT remove this delay
        // Delay to prevent the loop from running too fast and to allow other tasks to run 
        pros::delay(10);
        
    }

}
