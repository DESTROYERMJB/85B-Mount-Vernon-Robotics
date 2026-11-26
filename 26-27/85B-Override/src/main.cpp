#include "main.h"
#include "hardware/Motor/Motor.hpp"
#include "pros/misc.h"
#include "robot.hpp"


// -1 = unselected
// Alliance: 0 = Red, 1 = Blue
// Routine:  0 = Left, 1 = Right, 2 = Skills (Red only)
int selected_alliance = -1;
int selected_routine = -1;

// UI object pointers needed inside callbacks
lv_obj_t * alliance_btnm = nullptr;
lv_obj_t * routine_btnm = nullptr;

// Global pointers for your telemetry text labels
lv_obj_t * x_label;
lv_obj_t * y_label;
lv_obj_t * heading_label;

// Button maps for LVGL 9 button matrices (always end with "")
static const char * alliance_map[] = {
    "Red", "Blue", ""
};

static const char * red_routine_map[] = {
    "Left", "Right", "\n",
    "Skills", ""
};

static const char * blue_routine_map[] = {
    "Left", "Right", ""
};

// Helper to clear the checked highlight on all buttons of a buttonmatrix
static void clear_btnm_selection(lv_obj_t * btnm, uint32_t btn_count) {
    for (uint32_t i = 0; i < btn_count; i++) {
        lv_buttonmatrix_clear_button_ctrl(btnm, i, LV_BUTTONMATRIX_CTRL_CHECKED);
    }
}

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

void screenSetup() {
    lv_obj_t * tabview = lv_tabview_create(lv_screen_active());
    lv_obj_t * auton_tab = lv_tabview_add_tab(tabview, "Auton Selector");
    lv_obj_t * telemetry_tab = lv_tabview_add_tab(tabview, "85B Telemetry");

    // 1. Alliance switch (top of Auton tab)
    alliance_btnm = lv_buttonmatrix_create(auton_tab);
    lv_buttonmatrix_set_map(alliance_btnm, alliance_map);
    lv_obj_set_size(alliance_btnm, 420, 50);
    lv_obj_align(alliance_btnm, LV_ALIGN_TOP_MID, 0, 8);
    lv_buttonmatrix_set_button_ctrl_all(alliance_btnm, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_one_checked(alliance_btnm, true);
    lv_obj_add_event_cb(alliance_btnm, alliance_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // 2. Routine switch (below Alliance switch; defaults to Red's map until tapped)
    routine_btnm = lv_buttonmatrix_create(auton_tab);
    lv_buttonmatrix_set_map(routine_btnm, red_routine_map);
    lv_obj_set_size(routine_btnm, 420, 115);
    lv_obj_align(routine_btnm, LV_ALIGN_TOP_MID, 0, 68);
    lv_buttonmatrix_set_button_ctrl_all(routine_btnm, LV_BUTTONMATRIX_CTRL_CHECKABLE);
    lv_buttonmatrix_set_one_checked(routine_btnm, true);
    lv_obj_add_event_cb(routine_btnm, routine_event_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    // Telemetry tab labels (keep your existing setup here)
    x_label = lv_label_create(telemetry_tab);
    lv_label_set_text(x_label, "X: 0.00");
    lv_obj_align(x_label, LV_ALIGN_TOP_LEFT, 20, 20);

    y_label = lv_label_create(telemetry_tab);
    lv_label_set_text(y_label, "Y: 0.00");
    lv_obj_align(y_label, LV_ALIGN_TOP_LEFT, 20, 60);

    heading_label = lv_label_create(telemetry_tab);
    lv_label_set_text(heading_label, "Heading: 0.00 deg");
    lv_obj_align(heading_label, LV_ALIGN_TOP_LEFT, 20, 100);
}

void initialize() {
    screenSetup();
    rightColor.set_led_pwm(100);
    leftColor.set_led_pwm(100);
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    chassis.calibrate();
    driveController.rumble("-..-");
    liftController.rumble("-..-");
}

void disabled() {}

void autonomous() {
    chassis.setBrakeMode(lemlib::BrakeMode::HOLD);
    
    // Require both an alliance and a routine to be selected
    if (selected_alliance == -1 || selected_routine == -1) {
        return;
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

bool matchLoad = false;
bool blocking = false;
bool spinForToggle = false;
bool spinForIntake = false;
bool spinReverse = false;
bool toggleRight = false;

bool toggleColorCheck (double hue) {
    if (selected_alliance == 0 && ((hue < 25 || hue > 335))) {
        return true;
    } else if (selected_alliance == 1 && (hue > 200 && hue < 250)) {
        return true;
    } else {
        return false;
    }
}

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


void interactions() {
    pros::Task intakeMotorTask(intakeMotorHandler);
    while (true) {
        if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && !matchLoad) {
            if(blocking) {
                blockerPiston.retract();
                blocking = false;
                pros::delay(50);
            }
            intakePiston.extend();
            matchLoad = true;
            intake.move(127);
            pros::delay(100);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && matchLoad) {
            intakePiston.retract();
            matchLoad = false;
            pros::delay(100);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT) && !blocking) {
            blockerPiston.extend();
            blocking = true;
            pros::delay(100);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT) && blocking) {
            blockerPiston.retract();
            blocking = false;
            pros::delay(100);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            spinForToggle = !spinForToggle;
            toggleRight = false;
            pros::delay(100);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            spinForToggle = !spinForToggle;
            toggleRight = true;
            pros::delay(100);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            spinForIntake = true;
            spinReverse = false;
            pros::delay(10);
        } else if (driveController.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            spinForIntake = true;
            spinReverse = true;
            pros::delay(10);
        } else {
            spinForIntake = false;
            spinReverse = false;
            pros::delay(10);
        }
        
    }
}

bool wristExtended = false;
bool clawClamped = false;

void placement() {
    while (true) {
        // Wrist toggle using built-in rising edge detection
        if (liftController.is_connected()) {
            if (liftController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
                if (!wristExtended) {
                    wrist.extend();
                    wristExtended = true;
                } else {
                    wrist.retract();
                    wristExtended = false;
                }
            }
        }

        // Claw toggle using built-in rising edge detection
        if (driveController.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1)) {
            if (!clawClamped) {
                claw.extend();
                clawClamped = true;
            } else {
                claw.retract();
                clawClamped = false;
            }
        }

        pros::delay(10);
    }
}
// Pass your actual tracking variables into this function
void update_telemetry_ui(double current_x, double current_y, double current_heading) {
    // lv_label_set_text_fmt allows you to format strings exactly like printf
    lv_label_set_text_fmt(x_label, "X: %.2f in", current_x);
    lv_label_set_text_fmt(y_label, "Y: %.2f in", current_y);
    lv_label_set_text_fmt(heading_label, "Heading: %.2f deg", current_heading);
}

void opcontrol() {
    chassis.setBrakeMode(lemlib::BrakeMode::COAST);
    pros::Task InteractionsTask(interactions);
    pros::Task placementTask(placement);
    while (true) {
    if(liftController.is_connected()){
        if (liftController.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            lift.move(127);
        } else if (liftController.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            lift.move(-127);
        } else {
            lift.move(0);
        }
    }
        chassis.driverControl(driveController);
        update_telemetry_ui(chassis.getX(), chassis.getY(), chassis.getHeading());
        pros::delay(10);
        
    }

}
