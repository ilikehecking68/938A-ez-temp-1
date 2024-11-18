#include "main.h"

pros::Motor intake(21);
pros::Motor pto(14);
pros::ADIDigitalOut pto_piston(1, false);
bool doinker_status = false;
pros::ADIDigitalOut doinker(2, false);
bool mogo_status = false;
pros::ADIDigitalOut mogo(3, false);


/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {11, -12, -13},  // Left Chassis Ports (negative port will reverse it!)
    {18, 19, -20},  // Right Chassis Ports (negative port will reverse it!)

    15,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // Print our branding over your terminal :D
  ez::ez_template_print();

  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(true);  // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0);    // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0, 0);     // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  tuner();

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(".");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // . . .
}

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
  // . . .
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
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */


void doinker_update(bool button_new_press){
  if (button_new_press){
    doinker_status = !doinker_status;
    doinker.set_value(doinker_status);
  }
}

void mogo_update(bool button_new_press){
  if (button_new_press){
    mogo_status = !mogo_status;
    mogo.set_value(mogo_status);
  }
}

void pto_piston_update(bool intake_in_button_new_press, bool intake_out_button_new_press, bool arm_up_button_new_press, bool arm_down_button_neW_press){
  if (intake_in_button_new_press || intake_out_button_new_press){
    pto_piston.set_value(false);
  } else if (arm_up_button_new_press || arm_down_button_neW_press){
    pto_piston.set_value(true);
  }
}

void intake_update(bool in_button_held, bool out_button_held){
  if (in_button_held){
    intake.move(127);
    pto.move(127);
  } else if (out_button_held){
    intake.move(-127);
    pto.move(-127);
  } else {
    intake.move(0);
    pto.move(0);
  }
}

void arm_update(bool up_button_new_press, bool down_button_new_press){
  if (up_button_new_press){
    pto.move(127);
  } else if (down_button_new_press){
    pto.move(-127);
  } else {
    pto.move(0);
  }
}

void opcontrol() {
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;

  chassis.drive_brake_set(driver_preference_brake);

  while (true) {
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!pros::competition::is_connected()) {
      // Enable / Disable PID Tuner
      //  When enabled:
      //  * use A and Y to increment / decrement the constants
      //  * use the arrow keys to navigate the constants
      if (master.get_digital_new_press(DIGITAL_X))
        chassis.pid_tuner_toggle();

      // Trigger the selected autonomous routine
      if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
        autonomous();
        chassis.drive_brake_set(driver_preference_brake);
      }

      chassis.pid_tuner_iterate();  // Allow PID Tuner to iterate
    }

    //chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade
    pto_piston_update(master.get_digital_new_press(DIGITAL_R2), master.get_digital_new_press(DIGITAL_R1), master.get_digital_new_press(DIGITAL_L1), master.get_digital_new_press(DIGITAL_L2));
    intake_update(master.get_digital(DIGITAL_R2), master.get_digital(DIGITAL_R1));
    arm_update(master.get_digital(DIGITAL_L1), master.get_digital(DIGITAL_L2));
    mogo_update(master.get_digital_new_press(DIGITAL_A));
    doinker_update(master.get_digital_new_press(DIGITAL_B));


    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}