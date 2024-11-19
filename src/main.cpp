#include "main.h"

pros::Motor pto(14, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
pros::Motor intake(21, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup intake_group({14, 21}, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
bool pto_piston_status = false;
pros::ADIDigitalOut pto_piston(2, false);
bool doinker_status = false;
pros::ADIDigitalOut doinker(1, false);
bool mogo_status = false;
pros::ADIDigitalOut mogo(4, false);


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
  pto.set_brake_mode(pros::MotorBrake::hold);
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

    //chassis.pid_drive_set(40_in, DRIVE_SPEED);
  //chassis.pid_wait();

  //chassis.pid_turn_set(90_deg, TURN_SPEED);
  //chassis.pid_wait();

  //chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  //chassis.pid_wait();

  chassis.pid_drive_set(-23, 120);
  chassis.pid_wait();
  //put down mogo
  //turn on intake

  //chassis.pid_swing_set(ez::LEFT_SWING, 180, 120, 5);
  //chassis.pid_wait();
  
  chassis.pid_turn_set(140, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(26, 50);
  chassis.pid_wait();

  chassis.pid_drive_set(-6, 120);
  chassis.pid_wait();

  chassis.pid_turn_set(127, 110);
  chassis.pid_wait();

  chassis.pid_drive_set(12, 50);
  chassis.pid_wait();

  chassis.pid_drive_set(-15, 120);
  chassis.pid_wait();

  chassis.pid_turn_set(37, 110);
  chassis.pid_wait();

  chassis.pid_drive_set(46, 95);
  chassis.pid_wait();

  //doinker out

  chassis.pid_turn_set(95, 50);
  chassis.pid_wait();

  chassis.pid_drive_set(-5, 70);
  chassis.pid_wait();

  chassis.pid_turn_set(145, 127);
  chassis.pid_wait();
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

void set_pto_piston(bool new_status){
  pto_piston_status = new_status;
  pto_piston.set_value(pto_piston_status);
}

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

void intake_update(bool in_button_held, bool out_button_held){
  if ((in_button_held || out_button_held) && pto_piston_status){
    set_pto_piston(false);
  }
  if (in_button_held){
    intake_group.move(127);
  } else if (out_button_held){
    intake_group.move(-127);
  } else {
    intake_group.move(0);
  }
}

enum arm_position {
  arm_loading = -210/*loading position*/,
  arm_scoring = 1/*scoring position*/,
  arm_noninterfere = 0 /*lowest position(should be 0)*/
};

arm_position arm_status = arm_noninterfere;

void arm_set_goal_position(arm_position goal_position){
  arm_status = goal_position;
  pto.move_absolute(arm_status, 600);
}

void arm_update(bool load_button_new_press, bool score_button_new_press, bool noninterfere_new_press){
  if (load_button_new_press || score_button_new_press || noninterfere_new_press){
    intake_group.move(0);
    if (!pto_piston_status){
      set_pto_piston(false);
    }
  }
  if (load_button_new_press){
    arm_set_goal_position(arm_loading);
  } else if (score_button_new_press){
    arm_set_goal_position(arm_scoring);
  } else if (noninterfere_new_press){
    arm_set_goal_position(arm_noninterfere);
  }
  if (pto.get_position() > arm_status + 5 && pto.get_position() < arm_status - 5 && arm_status == arm_scoring){
    arm_set_goal_position(arm_loading);
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
    intake_update(master.get_digital(DIGITAL_R2), master.get_digital(DIGITAL_R1));
    arm_update(master.get_digital_new_press(DIGITAL_L1), master.get_digital_new_press(DIGITAL_L2), master.get_digital_new_press(DIGITAL_UP));
    mogo_update(master.get_digital_new_press(DIGITAL_A));
    doinker_update(master.get_digital_new_press(DIGITAL_B));


    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}