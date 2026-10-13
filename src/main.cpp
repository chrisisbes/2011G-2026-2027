#include "main.h"
#include <sys/types.h>
#include "autons.hpp"
#include "globals.hpp"
#include "okapi/api/units/QTime.hpp"
#include "pros/misc.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/screen.hpp"
#include "subsystems.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////


bool debug = true;
bool arm = false;
bool arm2 = false;
bool spinIntake = false;
bool sweeper = false;
int armToggle = 0;
bool stacker = false;
bool sortRed = false;
int seeRed = 0;
int seeBlue = 0;
bool DRIVE_SKILLS = true;

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-12, -16, -13},  // Left Chassis Ports (negative port will reverse it!)
    {11, 15, 14},  // Right Chassis Ports (negative port will reverse it!)

    7,      // IMU Port
    2.75,   // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450.0);  // Wheel RPM = cartridge * (motor gear / wheel gear)

// Uncomment the trackers you're using here!
// - `8` and `9` are smart ports (making these negative will reverse the sensor)
//  - you should get positive values on the encoders going FORWARD and RIGHT
// - `2.75` is the wheel diameter
// - `4.0` is the distance from the center of the wheel to the center of the robot
ez::tracking_wheel horiz_tracker(20, 2.75, 1.625);  // This tracking wheel is perpendicular to the drive wheels
ez::tracking_wheel vert_tracker(19, 2.75, 0.8125);   // This tracking wheel is parallel to the drive wheels







/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  // pros::Task intake_task(intake_control_task, NULL, "Intake Control task");
  Rotation.reset_position(); // Resets Rotation Sensor Value --------------------------------------------------------------------------------
  Rotation.set_data_rate(5); // Sets Data Rate to 5ms (fastest possible refresh) -----------------------------------------------------
  LiftLeft.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  LiftRight.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  lift_controller.exit_condition_set(80, 50, 300, 150, 500, 500);
  ez::ez_template_print();




  pros::delay(500);  // Stop the user from doing anything while legacy ports configure

  // Look at your horizontal tracking wheel and decide if it's in front of the midline of your robot or behind it
  //  - change `back` to `front` if the tracking wheel is in front of the midline
  //  - ignore this if you aren't using a horizontal tracker
  // chassis.odom_tracker_back_set(&horiz_tracker);
  // Look at your vertical tracking wheel and decide if it's to the left or right of the center of the robot
  //  - change `left` to `right` if the tracking wheel is to the right of the centerline
  //  - ignore this if you aren't using a vertical tracker
  // chassis.odom_tracker_left_set(&vert_tracker);

  // Configure your chassis controls
  chassis.opcontrol_curve_buttons_toggle(false);   // Enables modifying the controller curve with buttons on the joysticks
  chassis.opcontrol_drive_activebrake_set(0.0);   // Sets the active brake kP. We recommend ~2.  0 will disable.
  chassis.opcontrol_curve_default_set(0.0, 0.0);  // Defaults for curve. If using tank, only the first parameter is used. (Comment this line out if you have an SD card!)

  // Set the drive to your own constants from autons.cpp!
  default_constants();

  // These are already defaulted to these buttons, but you can change the left/right curve buttons here!
  // chassis.opcontrol_curve_buttons_left_set(pros::E_CONTROLLER_DIGITAL_LEFT, pros::E_CONTROLLER_DIGITAL_RIGHT);  // If using tank, only the left side is used.
  // chassis.opcontrol_curve_buttons_right_set(pros::E_CONTROLLER_DIGITAL_Y, pros::E_CONTROLLER_DIGITAL_A);

  // Autonomous Selector using LLEMU
  ez::as::auton_selector.autons_add({
      {"", near_side},
      {"", swing_example},
      {"", motion_chaining},
      {"", combining_movements},
      {"", interfered_example},
      {"", odom_drive_example},
      {"", odom_pure_pursuit_example},
      {"", odom_pure_pursuit_wait_until_example},
      {"", odom_boomerang_example},
      {"", odom_boomerang_injected_pure_pursuit_example},
      {"", measure_offsets},
  });

  // Initialize chassis and auton selector
  chassis.initialize();
  ez::as::initialize();
  master.rumble(chassis.drive_imu_calibrated() ? "." : "---");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
// void disabled() {
//   // . . .
//   Arm.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
//   Arm5.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);


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


 void ez_screen_task() {
  while (true) {
    pros::screen::print(
        TEXT_MEDIUM,
        4,
        "X: %.2f in       ",
        chassis.odom_x_get()
    );

    pros::screen::print(
        TEXT_MEDIUM,
        5,
        "Y: %.2f in       ",
        chassis.odom_y_get()
    );

    pros::screen::print(
        TEXT_MEDIUM,
        6,
        "Heading: %.2f deg       ",
        chassis.odom_theta_get()
    );

    pros::delay(50);
  }
}
void autonomous() {
  pros::screen::print(TEXT_MEDIUM, 7,"Lift: %.2f deg",Rotation.get_position() / 100.0);
  chassis.pid_targets_reset();                // Resets PID targets to 0
  chassis.drive_imu_reset();                  // Reset gyro position to 0
  chassis.drive_sensor_reset();               // Reset drive sensors to 0
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);    // Set the current position
  chassis.drive_brake_set(MOTOR_BRAKE_HOLD);  // Hold mode for consistency
  start_lift_pid_task();
  ez::as::auton_selector.selected_auton_call();
  pros::screen::print(TEXT_MEDIUM, 6, "Heading: %.2f", chassis.odom_theta_get());
  pros::screen::print(TEXT_MEDIUM, 7,"Lift: %.2f deg",Rotation.get_position() / 100.0);

}

/**
 * Simplifies printing tracker values to the brain screen
 */
void screen_print_tracker(ez::tracking_wheel *tracker, std::string name, int line) {
  std::string tracker_value = "", tracker_width = "";
  // Check if the tracker exists
  if (tracker != nullptr) {
    tracker_value = name + " tracker: " + util::to_string_with_precision(tracker->get());             // Make text for the tracker value
    tracker_width = "  width: " + util::to_string_with_precision(tracker->distance_to_center_get());  // Make text for the distance to center
  }
  ez::screen_print(tracker_value + tracker_width, line);  // Print final tracker text
}

/**
 * Ez screen task
 * Adding new pages here will let you view them during user control or autonomous
 * and will help you debug problems you're having
 */




/**
 * Gives you some extras to run in your opcontrol:
 * - run your autonomous routine in opcontrol by pressing DOWN and B
 *   - to prevent this from accidentally happening at a competition, this
 *     is only enabled when you're not connected to competition control.
 * - gives you a GUI to change your PID values live by pressing X
 */
void ez_template_extras() {
  if (DRIVE_SKILLS) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp


    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_B) && master.get_digital(DIGITAL_DOWN)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }
  
  
  // Only run this when not connected to a competition switch
  if (!pros::competition::is_connected()) {
    // PID Tuner
    // - after you find values that you're happy with, you'll have to set them in auton.cpp

    // Enable / Disable PID Tuner
    //  When enabled:
    //  * use A and Y to increment / decrement the constants
    //  * use the arrow keys to navigate the constants
    if (master.get_digital_new_press(DIGITAL_X))
      chassis.pid_tuner_toggle();

    // Trigger the selected autonomous routine
    if (master.get_digital(DIGITAL_X) && master.get_digital(DIGITAL_UP)) {
      pros::motor_brake_mode_e_t preference = chassis.drive_brake_get();
      autonomous();
      chassis.drive_brake_set(preference);
    }

    // Allow PID Tuner to iterate
    chassis.pid_tuner_iterate();
  }

  // Disable PID Tuner when connected to a comp switch
  else {
    if (chassis.pid_tuner_enabled())
      chassis.pid_tuner_disable();
  }
}
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
void opcontrol() {
  // This is preference to what you like to drive on
  chassis.drive_brake_set(MOTOR_BRAKE_COAST);
  autonomous_mode=false;
  autonomous_mode = false; 
  reverse_on_red=false; 
  
  // Track PID
  chassis.pid_tuner_print_terminal_set(true);

  chassis.pid_tuner_increment_p_set(0.1);
  chassis.pid_tuner_increment_i_set(0.01);
  chassis.pid_tuner_increment_d_set(0.5);
  chassis.pid_tuner_increment_start_i_set(5);
  
  //  start_lift_pid_task();
  stop_lift_pid();
  while (true) {
    // Gives you some extras to make EZ-Template ezier
    ez_template_extras();

    chassis.drive_set(
    -master.get_analog(ANALOG_RIGHT_Y),
    -master.get_analog(ANALOG_LEFT_Y)); //Backward Tank
    // chassis.opcontrol_tank();  // Tank control
    // chassis.opcontrol_arcade_standard(ez::SPLIT);   // Standard split arcade
    // chassis.opcontrol_arcade_standard(ez::SINGLE);  // Standard single arcade
    // chassis.opcontrol_arcade_flipped(ez::SPLIT);    // Flipped split arcade
    // chassis.opcontrol_arcade_flipped(ez::SINGLE);   // Flipped single arcade

    // . . .
    // Put more user control code here!
    // . . .
// // Arm Function -------------------------------------------------------------------------------------------------
   
    // Old Toggle Function
    // if (master.get_digital_new_press(DIGITAL_L1)){
    //   ++armToggle;
    //   if(armToggle > 2){
    //     armToggle = 0;
    //   }
    // }
   
    // if (armToggle == 0){
    //   set_lift_pos(1500, 1000);
    // }
    // else if (armToggle == 1){
    //   set_lift_pos(3650, 1000);
    // }
    // else if (armToggle == 2){
    //   set_lift_pos(13500, 1000);
    // }


    
     

    if (master.get_digital(DIGITAL_R1)){
      Claw.move(-120);
    }
    else if (master.get_digital(DIGITAL_R2)){
      Claw.move(120); //Open
    }
    else{
      Claw.move(0);
    }

//    Lift

 if (master.get_digital(DIGITAL_L1) && get_Lift_position() < 1000) {


  // lift up

    LiftLeft.move(127);
  LiftRight.move(127);

}
else if (master.get_digital(DIGITAL_L2)) {

  // lift down

  LiftLeft.move(-80);
  LiftRight.move(-80);
}
else {

  LiftLeft.move(0);
  LiftRight.move(0);
  }


//hi
    

    
    if (master.get_digital_new_press(DIGITAL_A)) {
        master.rumble("._.");
    }



  

    // Debug Functions ----------------------------------------------------------------------------------------------------------------------

    // Sets the _temp variables to their respective motor temps (averages if multiple motors)
    int intake_temp = (pros::c::motor_get_temperature(9));
    int drive_temp = (pros::c::motor_get_temperature(12) + pros::c::motor_get_temperature(16) + pros::c::motor_get_temperature(13) + pros::c::motor_get_temperature(11) + pros::c::motor_get_temperature(15) + pros::c::motor_get_temperature(14)) / 6;
    int right_drive_temp = (pros::c::motor_get_temperature(11) + pros::c::motor_get_temperature(15) + pros::c::motor_get_temperature(14)) / 3;
    int left_drive_temp = (pros::c::motor_get_temperature(12) + pros::c::motor_get_temperature(16) + pros::c::motor_get_temperature(13)) / 3;

    //Prints values for debugging if bool debug is set to true (at the top of the code)
    if (debug == true) {
        pros::screen::print(TEXT_MEDIUM, 1, "Drive: %d", drive_temp);
        pros::screen::print(TEXT_MEDIUM, 2, "Right Drive: %d", right_drive_temp);
        pros::screen::print(TEXT_MEDIUM, 3, "Left Drive: %d", left_drive_temp);
        pros::screen::print(TEXT_MEDIUM, 4, "Left Drive: %d", left_drive_temp);
        pros::screen::print(TEXT_MEDIUM, 5, "Intake: %d", intake_temp);
        pros::screen::print(TEXT_MEDIUM, 6, "Heading: %.2f", chassis.odom_theta_get());
        pros::screen::print(TEXT_MEDIUM, 7,"Lift: %.2f deg",Rotation.get_position() / 100.0);
    }


    // Always prints the temperatures of each motor group to the controller
    master.print(0,10,"I: %d", intake_temp);
    master.print(0,0,"D: %d", drive_temp);
    pros::screen::print(TEXT_MEDIUM, 6, "Heading: %.2f", chassis.odom_theta_get());

    pros::delay(ez::util::DELAY_TIME);  // This is used for timer calculations!  Keep this ez::util::DELAY_TIME
  } // closes while loop
} // closes opcontrol Sqarika 