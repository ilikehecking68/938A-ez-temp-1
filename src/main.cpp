#include "main.h"
#include <sys/_intsup.h>
#include "EZ-Template/util.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"

pros::MotorGroup intake({-4, 21}, pros::MotorGears::blue, pros::MotorEncoderUnits::degrees);
bool doinker_status = false;
bool prev_doinker_status = false;
#define doinker_out true
#define doinker_in false
pros::ADIDigitalOut doinker(1, false);
bool mogo_status = false;
#define mogo_up true
#define mogo_down false
pros::ADIDigitalOut mogo(4, false);
pros::Motor arm_motor(3, pros::MotorGears::green, pros::MotorEncoderUnits::degrees);
pros::Rotation arm_sensor(9);
#define arm_sensor_get_degrees()((double)((arm_sensor).get_position() / 100))


void doinker_update(bool button_new_press){
  if (button_new_press && !mogo_status){ //runs if button is pressed(not held)
    doinker_status = !doinker_status; //toggle doinker status
  }
  if (mogo_status){
    doinker_status = false;
  }
  if (doinker_status != prev_doinker_status){
    doinker.set_value(doinker_status);
  }
  prev_doinker_status = doinker_status;
}

void mogo_update(bool button_new_press){
  if (button_new_press){ //runs if button is pressed(not held)
    mogo_status = !mogo_status; //toggle mog status
    mogo.set_value(mogo_status); //set mogo
  }
}

#define arm_loading 26/*loading position*/
#define arm_noninterfere 0 /*lowest position(should be 0)*/
#define arm_scoring 154

double arm_target = arm_noninterfere;

ez::PID arm_pid(3, 0, 0, 0, "Arm");
bool arm_pid_running = false;
/*bool arm_is_scoring = false;
unsigned long long arm_scoring_time = 0;*/
void arm_update(bool up_button_held, bool load_button_new_press, bool noninterfere_button_new_press){

  /*if (up_button_held){ //to score
    arm_motor.move(-127); //move arm up
    arm_pid_running = false;*/

  /*int wallstakearm = 0;//0:neutral, 1:loading, 2:scoring
  if (up_button_held){
    if (wallstakearm == 0){
      arm_pid_running = false;
      arm_is_scoring = true;
      wallstakearm++;
    } else if (wallstakearm == 1) {
      arm_target = arm_loading;
      arm_pid.target_set(arm_target);
      arm_pid_running = true;
      arm_is_scoring = false;
      wallstakearm++;
    } else {
      arm_target = arm_noninterfere;
      arm_pid.target_set(arm_target);
      arm_pid_running = true;
      arm_is_scoring = false;
      if (wallstakearm >= ){}
      wallstakearm = 0;
    }
  }*/
  /*if (up_button_held){
    arm_motor.move(-127);
    arm_pid_running = false;
  } else if (load_button_new_press){
    arm_target = arm_loading;
    arm_pid.target_set(arm_target);
    arm_pid_running = true;
  } else if (noninterfere_button_new_press){
    arm_target = arm_noninterfere;
    arm_pid.target_set(arm_target);
    arm_pid_running = true;
  }*/

  /*if (arm_pid_running){
    arm_motor.move(-(arm_pid.compute(arm_sensor_get_degrees()))); //compute PID for arm and set it motor value to it
  } else if (!up_button_held){
    arm_motor.move(0); //if pid not run and not scoring stop arm motor
    arm_motor.move(-(arm_pid.compute(arm_sensor_get_degrees())));
  } else if (!up_button_held/!arm_is_scoring && arm_scoring_time > 100*//*){
    arm_motor.move(0);
  }*/
}

void intake_update(bool in_button_held, bool out_button_held){
  if (in_button_held){
    intake.move(127);
  } else if (out_button_held){
    intake.move(-127);
  } else {
    intake.move(0);
  }
}

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

void arm_mtp(double degrees){
  arm_pid.target_set(degrees);
  while (arm_pid.exit_condition() != ez::SMALL_EXIT){
    arm_motor.move(-(arm_pid.compute(arm_sensor_get_degrees())));
  }
}

void redright(){
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  doinker.set_value(doinker_in);
  mogo.set_value(mogo_up);

  chassis.pid_drive_set(-15, 95);
  chassis.pid_wait();

  chassis.pid_turn_set(35, 80);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-20, 50);
  chassis.pid_wait();

  pros::delay(5);

  mogo.set_value(mogo_down);

  pros::delay(500);

  chassis.pid_turn_set(-92, 100);
  chassis.pid_wait();

  intake.move(-127);

  chassis.pid_drive_set(30, 80);
  chassis.pid_wait();

};

void redleft_2ring_bar_touch(){
  doinker.set_value(doinker_in);
  mogo.set_value(mogo_up);

  chassis.pid_drive_set(-15, 95);
  chassis.pid_wait();

  chassis.pid_turn_set(-35, 80);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-20, 50);
  chassis.pid_wait();

  pros::delay(5);

  mogo.set_value(mogo_down);

  pros::delay(500);

  chassis.pid_turn_set(90, 100);
  chassis.pid_wait();

  intake.move(-127);

  chassis.pid_drive_set(20, 80);
  chassis.pid_wait();

  chassis.pid_turn_set(-90, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(55, 100);
  chassis.pid_wait();
};

void redleft_4_ring(){
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  //chassis.pid_turn_set(90_deg, TURN_SPEED);
  //chassis.pid_wait();

  //chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
  //chassis.pid_wait();

  doinker.set_value(doinker_in);
  mogo.set_value(mogo_up);

  chassis.pid_drive_set(-15, 95);
  chassis.pid_wait();

  chassis.pid_turn_set(-35, 80);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-20, 50);
  chassis.pid_wait();

  pros::delay(5);

  mogo.set_value(mogo_down);

  pros::delay(500);

  chassis.pid_turn_set(90, 100);
  chassis.pid_wait();

  intake.move(-127);

  chassis.pid_drive_set(20, 80);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(14, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(-8, 120);
  chassis.pid_wait();

  chassis.pid_turn_set(90, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(6, 100);
  chassis.pid_wait();

  chassis.pid_turn_set(180, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(8, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(-25, 50);
  chassis.pid_wait();

  chassis.pid_turn_set(-90, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(50, 100);
  chassis.pid_wait();

  /*chassis.pid_drive_set(-30, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(38, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(25.5, 120);
  chassis.pid_wait();

  doinker.set_value(doinker_out);

  chassis.pid_turn_set(-25, 120);
  chassis.pid_wait();

  doinker.set_value(doinker_in);

  chassis.pid_turn_set(-135, 120);
  chassis.pid_wait();

  chassis.pid_drive_set(50, 127);
  chassis.pid_wait();*/
}

void blueright_4ring() {

  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  doinker.set_value(doinker_in);
  mogo.set_value(mogo_up);

  chassis.pid_drive_set(-15, 95);
  chassis.pid_wait();

  chassis.pid_turn_set(35, 80);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-20, 50);
  chassis.pid_wait();

  pros::delay(5);

  mogo.set_value(mogo_down);

  pros::delay(500);

  chassis.pid_turn_set(-90, 100);
  chassis.pid_wait();

  intake.move(-127);

  chassis.pid_drive_set(20, 80);
  chassis.pid_wait();

  chassis.pid_turn_set(-180, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(14, 60);
  chassis.pid_wait();

  chassis.pid_drive_set(-6, 120);
  chassis.pid_wait();

  chassis.pid_turn_set(-90, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(6, 50);
  chassis.pid_wait();

  chassis.pid_turn_set(-180, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(6.5, 80);
  chassis.pid_wait();

  chassis.pid_drive_set(-40, 100);
  chassis.pid_wait();

  /*chassis.pid_drive_set(-30, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(-38, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(25.5, 120);
  chassis.pid_wait();

  doinker.set_value(doinker_out);

  chassis.pid_turn_set(25, 120);
  chassis.pid_wait();

  doinker.set_value(doinker_in);

  chassis.pid_turn_set(150, 120);
  chassis.pid_wait();

  chassis.pid_drive_set(54, 127);
  chassis.pid_wait();*/

}

void sawp_red_local(){
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  doinker.set_value(doinker_in);
  mogo.set_value(mogo_up);

  chassis.pid_drive_set(-35, 120);
  chassis.pid_wait();

  chassis.pid_turn_set(25, 110);
  chassis.pid_wait();

  chassis.pid_drive_set(-20, 70);
  chassis.pid_wait();

  pros::delay(5);

  mogo.set_value(mogo_down);

  pros::delay(500);

  chassis.pid_turn_set(-2, 100);
  chassis.pid_wait();

  intake.move(-127);

  pros::delay(1000);

  chassis.pid_drive_set(3, 127);
  chassis.pid_wait();
  chassis.pid_drive_set(-3, 127);
  chassis.pid_wait();

  mogo.set_value(mogo_down);

  chassis.pid_drive_set(20, 80);
  chassis.pid_wait();

  pros::delay(200);

  intake.move(0);

  chassis.pid_turn_set(-90, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(-20, 80);
  chassis.pid_wait();

  pros::delay(5);

  mogo.set_value(mogo_down);

  pros::delay(500);

  intake.move(-127);

  chassis.pid_drive_set(3, 127);
  chassis.pid_wait();
  chassis.pid_drive_set(-3, 127);
  chassis.pid_wait();

  chassis.pid_turn_set(90, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(10, 100);
  chassis.pid_wait();
}

void blueleft(){
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  doinker.set_value(doinker_in);
  mogo.set_value(mogo_up);

  chassis.pid_drive_set(-15, 95);
  chassis.pid_wait();

  chassis.pid_turn_set(-35, 80);
  chassis.pid_wait();
  
  chassis.pid_drive_set(-20, 50);
  chassis.pid_wait();

  pros::delay(5);

  mogo.set_value(mogo_down);

  pros::delay(500);

  chassis.pid_turn_set(92, 100);
  chassis.pid_wait();

  intake.move(-127);

  chassis.pid_drive_set(30, 80);
  chassis.pid_wait();

  chassis.pid_turn_set(-90, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(45, 127);
  chassis.pid_wait();

  /*chassis.pid_turn_set(55, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(-35, 100);
  chassis.pid_wait();*/
};

void one_inch_sawp_coop(){
  chassis.pid_drive_set(1, 100);
  chassis.pid_wait();
}

void sawp(){
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  doinker.set_value(doinker_in);
  mogo.set_value(mogo_up);

  chassis.pid_drive_set(-50, 120);
  chassis.pid_wait();

  chassis.pid_turn_set(35, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(-7, 60);
  chassis.pid_wait();

  pros::delay(5);

  mogo.set_value(mogo_down);

  pros::delay(500);

  intake.move(-127); // turn on intake to score preload

  pros::delay(800); // wait till intake is done scoring

  intake.move(0); // stop intake

  chassis.pid_drive_set(13, 120);
  chassis.pid_wait();
  
  mogo.set_value(mogo_up);

  chassis.pid_turn_set(40, 120);
  chassis.pid_wait();

  chassis.pid_drive_set(-28, 100);
  chassis.pid_wait();

  chassis.pid_drive_set(-5, 50);
  chassis.pid_wait();

  pros::delay(5);

  mogo.set_value(mogo_down);

  chassis.pid_turn_set(-90, 100);
  chassis.pid_wait();

  intake.move(-127);

  chassis.pid_drive_set(20, 100);
  chassis.pid_wait();

  intake.move(0);

  chassis.pid_turn_set(55, 110);
  chassis.pid_wait();

  //raise arm to loading pose

  intake.move(-60);

  chassis.pid_drive_set(40, 100); // go intake the bottom blue ring of the stack
  chassis.pid_wait();

  pros::delay(80);

  chassis.pid_turn_set(-85, 127);
  chassis.pid_wait();

  intake.move(127);

  pros::delay(750);

  chassis.pid_turn_set(55, 120);
  chassis.pid_wait();


};

void initialize() {
  // Print our branding over your terminal :D
  arm_motor.set_brake_mode(pros::MotorBrake::hold);
  arm_motor.tare_position_all();
  arm_sensor.reverse();
  arm_sensor.reset_position();
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
  ez::as::auton_selector.autons_add({
      Auton("red right", redright),
      Auton("red left 4", redleft_4_ring),
      Auton("blue right 4", blueright_4ring),
      Auton("blue left", blueleft),
      Auton("red local SAWP", sawp_red_local)
  });

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

void autonomous() {
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Set motors to hold.  This helps autonomous consistency

  ez::as::auton_selector.selected_auton_call();  // Calls selected auton from autonomous selector
}


void opcontrol() {
  // This is preference to what you like to drive on
  pros::motor_brake_mode_e_t driver_preference_brake = MOTOR_BRAKE_COAST;

  chassis.drive_brake_set(driver_preference_brake);
  arm_pid.exit_condition_set(80, 2, 250, 8, 500, 500);
  while (true) {
    
    // PID Tuner
    // After you find values that you're happy with, you'll have to set them in auton.cpp
    if (!(pros::competition::is_connected())){
      if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)){
        autonomous();
      }
    }
    /*if (!pros::competition::is_connected()) {
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
    }*/

    //chassis.opcontrol_tank();  // Tank control
    chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade
    intake_update(master.get_digital(DIGITAL_R2), master.get_digital(DIGITAL_R1));
    arm_update(master.get_digital(DIGITAL_B), master.get_digital_new_press(DIGITAL_RIGHT), master.get_digital_new_press(DIGITAL_DOWN));
    mogo_update(master.get_digital_new_press(DIGITAL_A));
    doinker_update(master.get_digital_new_press(DIGITAL_UP));
    //arm_update(master.get_digital(DIGITAL_B), master.get_digital_new_press(DIGITAL_RIGHT), master.get_digital_new_press(DIGITAL_DOWN));
    mogo_update(master.get_digital_new_press(DIGITAL_A));
    doinker_update(master.get_digital_new_press(DIGITAL_L1));
    pros::lcd::print(3, "%lf", (double)(arm_sensor.get_position() / 100));
    
    // . . .
    // Put more user control code here!
    // . . .

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  }
}