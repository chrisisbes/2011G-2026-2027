#include <sys/_intsup.h>
#include "EZ-Template/drive/drive.hpp"
#include "EZ-Template/util.hpp"
#include "main.h"
#include "pros/motors.h"
#include "pros/rtos.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"
#include "globals.hpp"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////

// These are out of 127
const int DRIVE_SPEED = 100;
const int TURN_SPEED = 120;
const int SWING_SPEED = 120;

///
// Constants
///

void default_constants() {
 // P, I, D, and Start I
 chassis.pid_drive_constants_set(13.7, 0.0,96.0); // Fwd/rev constants, used for odom and non odom motions
 chassis.pid_heading_constants_set(0.8, 0.0, 0.3); // Holds the robot straight while going forward without odom
 chassis.pid_turn_constants_set(1.00, 0.0, 2.75); // Turn in place constants
 chassis.pid_swing_constants_set(6.0, 0.0, 65.0); // Swing constants
 chassis.pid_odom_angular_constants_set(4.8, 0.0, 50.0); // Angular control for odom motions
 chassis.pid_odom_boomerang_constants_set(6.0, 0.0, 48.0); // Angular control for boomerang motions

 // Exit conditions
chassis.pid_turn_exit_condition_set(40_ms, 4_deg, 80_ms, 7_deg, 150_ms, 150_ms);
 chassis.pid_swing_exit_condition_set(30_ms, 2_deg, 70_ms, 6_deg, 70_ms, 90_ms);
chassis.pid_drive_exit_condition_set(40_ms, 2.5_in, 80_ms, 4_in, 150_ms, 150_ms);
 chassis.pid_odom_turn_exit_condition_set(70_ms, 3_deg, 100_ms, 7_deg, 250_ms, 250_ms);
 chassis.pid_odom_drive_exit_condition_set(70_ms, 2.5_in, 100_ms, 4_in, 250_ms, 250_ms);
 chassis.pid_turn_chain_constant_set(5_deg);
 chassis.pid_swing_chain_constant_set(13_deg);
 chassis.pid_drive_chain_constant_set(4_in);
 

 // Slew constants
 chassis.slew_turn_constants_set(3_deg, 70);
 chassis.slew_drive_constants_set(5_in, 80);
 chassis.slew_swing_constants_set(3_in, 80);

 // The amount that turns are prioritized over driving in odom motions
 // - if you have tracking wheels, you can run this higher. 1.0 is the max
 chassis.odom_turn_bias_set(0.81);
 
 chassis.odom_look_ahead_set(10_in); // This is how far ahead in the path the robot looks at
 chassis.odom_boomerang_distance_set(10_in); // This sets the maximum distance away from target that the carrot point can be
 chassis.odom_boomerang_dlead_set(0.600); // This handles how aggressive the end of boomerang motions are

 chassis.pid_angle_behavior_set(ez::shortest); // Changes the default behavior for turning, this defaults it to the shortest path there
}



// Void Methods for Auton ----------------------------------------------------------------------------------------------
// Task Methods --------------------------------------------------------------------------------------------------------



// Shortcut Methods ----------------------------------------------------------------------------------------------------
void drive(int target){
 chassis.pid_odom_set(target, DRIVE_SPEED, true);
 chassis.pid_wait();
}

void drive(double target, int speed){
 chassis.pid_odom_set(target, speed, true);
 chassis.pid_wait();
}

void odrive(int target, int speed){
 chassis.pid_odom_set(target, speed);
}

void turn(double target){
 chassis.pid_turn_set(target, TURN_SPEED, true);
 chassis.pid_wait();
}

void turn(double target, int speed){
 chassis.pid_turn_set(target, speed, true);
 chassis.pid_wait();
}

void lswing(int target){
 chassis.pid_swing_set(ez::LEFT_SWING, target, SWING_SPEED);
 chassis.pid_wait();
}

void lswing(int target, int speed){
 chassis.pid_swing_set(ez::LEFT_SWING, target, speed);
 chassis.pid_wait();
}

void lswing(int target, int chaintarget, int oppositeSpeed, int speed){
 chassis.pid_swing_set(ez::LEFT_SWING, target, speed, oppositeSpeed, true);
 chassis.pid_wait();
}

void chain_lswing(int target,int chaintarget){
 chassis.pid_swing_set(ez::LEFT_SWING, target, SWING_SPEED);
 chassis.pid_wait_until(chaintarget);
}

void chain_lswing(int target, int chaintarget, int oppositeSpeed){
 chassis.pid_swing_set(ez::LEFT_SWING, target, SWING_SPEED, oppositeSpeed, true);
 chassis.pid_wait_until(chaintarget);
}

void chain_lswing(int target, int chaintarget, int oppositeSpeed, int speed){
 chassis.pid_swing_set(ez::LEFT_SWING, target, speed, oppositeSpeed, true);
 chassis.pid_wait_until(chaintarget);
}

void rswing(int target){
 chassis.pid_swing_set(ez::RIGHT_SWING, target, SWING_SPEED);
 chassis.pid_wait();
}

void rswing(int target, int speed){
 chassis.pid_swing_set(ez::RIGHT_SWING, target, speed);
 chassis.pid_wait();
}

void rswing(int target, int speed, int oppositeSpeed){
 chassis.pid_swing_set(ez::RIGHT_SWING, target, speed, oppositeSpeed,true);
 chassis.pid_wait();
}

void rswing(int target, int chaintarget, int oppositeSpeed, int speed){
 chassis.pid_swing_set(ez::RIGHT_SWING, target, speed, oppositeSpeed,true);
 chassis.pid_wait();
}

void chain_rswing(int target, int chaintarget){
 chassis.pid_swing_set(ez::RIGHT_SWING, target, SWING_SPEED);
 chassis.pid_wait_until(chaintarget);
}

void chain_rswing(int target, int chaintarget, int oppositeSpeed){
 chassis.pid_swing_set(ez::RIGHT_SWING, target, SWING_SPEED, oppositeSpeed, true);
 chassis.pid_wait_until(chaintarget);
}

void chain_rswing(int target, int chaintarget, int oppositeSpeed, int speed){
 chassis.pid_swing_set(ez::RIGHT_SWING, target, speed, oppositeSpeed, false);
 chassis.pid_wait_until(chaintarget);
}

void chain_drive(double target){
 chassis.pid_odom_set(target, DRIVE_SPEED, true);
 chassis.pid_wait_quick_chain();
}

void chain_drive(double target, int speed){
 chassis.pid_odom_set(target, speed, true);
 chassis.pid_wait_quick_chain();
}

void chain_drive(double target, int chain_target, int speed){
 chassis.pid_odom_set(target, speed);
 chassis.pid_wait_until(chain_target);
}

void max_drive(double target, int speed){
 chassis.pid_drive_set(target, speed);
 chassis.pid_wait();
}

void mid_drive(int target, int chain_target){
 chassis.pid_drive_set(target, 80);
 chassis.pid_wait_until(chain_target);
}

void mid2_drive(int target, int chain_target){
 chassis.pid_drive_set(target, 0);
 chassis.pid_wait_until(chain_target);
}

void chain_turn(int target){
 chassis.pid_turn_set(target, TURN_SPEED, true);
 chassis.pid_wait_quick_chain();
}

void chain_turn(double target, int speed){
 chassis.pid_turn_set(target, speed, true);
 chassis.pid_wait_quick_chain();
}



// OBAMATRYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYYY ----------------------------------------------------------------------------------------

void omove(double x, double y, int direction){
 if (direction > 0){
 chassis.pid_odom_set({{x, y}, fwd, DRIVE_SPEED},
 false);
 chassis.pid_wait_quick_chain();
 }
 else{
 chassis.pid_odom_set({{x, y}, rev, DRIVE_SPEED},
 true);
 chassis.pid_wait_quick_chain();
 }
}

void omove(double x, double y, double angle, int direction){
 if (direction > 0){
 chassis.pid_odom_set({{x, y, angle}, fwd, DRIVE_SPEED},
 true);
 chassis.pid_wait_quick_chain();
 }
 else{
 chassis.pid_odom_set({{x, y, angle}, rev, DRIVE_SPEED},
 true);
 chassis.pid_wait_quick_chain();
 }
}

void omove(double x, double y, double angle, int direction, int speed){
 if (direction > 0){
 chassis.pid_odom_set({{x, y, angle}, fwd, speed},
 false);
 chassis.pid_wait_quick_chain();
 }
 else{
 chassis.pid_odom_set({{x, y, angle}, rev, speed},
 false);
 chassis.pid_wait_quick_chain();
 }
}


// --------------------------------------------------------------------------------------------------------------------------------

void raiseLift() {
    //Lift.move(127);
    pros::delay(600);   // tune this
    //Lift.move(0);
}
void lowLift() {
    //Lift.move(127);
    pros::delay(600);   // tune this
    //Lift.move(0);
}


 // *** Example Methods being called and how to use them!!!!!!! ***
 // pros::Task task1(wing_slap);
 // pros::Task stinky(intake_release);
 // chassis.pid_swing_set(ez::RIGHT_SWING, -38.5, 127, 60);
 // pros::delay(380);
 // drive(44.25);
 // turn(-40);
 // chassis.pid_turn_set(45_deg, TURN_SPEED, false);
 // chassis.pid_wait_until(10_deg);
 // intake_move(-127);
 // pros::delay(340);
 // chassis.pid_swing_set(ez::LEFT_SWING, -90, TURN_SPEED, -30);
 // chassis.pid_wait();




 // *** REMINDER FOR USING TASKS ***
 // If you want to use multiple tasks in a single auton,
 // you need to name each task differently, in number order
 // eg. pros::Task task1(something);
 // pros::Task task2(something else);
 // Even if it is the same void method you are running, must be a
 // different number when using it
 // Doesn't carry over between autons, so you can rename starting with task1
 // on the next one


///
// Interference example
///
void tug(int attempts) {
 for (int i = 0; i < attempts - 1; i++) {
 // Attempt to drive backward
 printf("i - %i", i);
 chassis.pid_odom_set(-12_in, 127);
 chassis.pid_wait();

 // If failsafed...
 if (chassis.interfered) {
 chassis.drive_sensor_reset();
 chassis.pid_odom_set(-2_in, 20);
 pros::delay(500);
 }
 // If the robot successfully drove back, return
 else {
 return;
 }
 }
}


void left_side(){
chassis.odom_xyt_set(0_in, 0_in, -135_deg); // Initialize the odometry with specified parameters
chain_drive(-5, 90);
//claw(true);
chain_drive(2, 80);
lswing(-180, -175, 5, 70);
chain_drive(2, 90);
chain_drive(-2, 90);
chain_drive(2, 90);
chain_drive(-24, 90);
//claw(false);
chain_turn(-135, 90);
chain_drive(-15, 90);
//claw(true);
chain_turn(0, 100);
pros::Task liftTask(raiseLift);
chain_drive(-18,100);
//claw(false);
rswing(45, 44, 5, 90);
pros::Task lowerTask(lowLift);
chain_drive(-30, 90);
//claw(true)
chain_turn(90, 100);
chain_drive(-24,90);
pros::Task liftTask2(raiseLift);
//claw(false)
chain_drive(-26, 90);
pros::Task lowerTask2(lowLift);
chain_turn(180);
chain_drive(-9, 30);
}


;

void pid_tuning_auton() {
turn(90, 80);

}


// Right Auton
void right_AWP(){
chassis.odom_xyt_set(12_in, 24_in, 90_deg); // Initialize the odometry with specified parameters
    default_constants();
// Loader.set_value(true);
drive(31,127); // Drive forward to between long goal and loader
chain_turn(180.5,90); // Turn to face loader
chain_drive(7.5, 70); 
pros::delay(400);
chain_drive(-26,100); // Drive back to score on long goal
pros::delay(1000); // Wait to score on long goal
// Loader.set_value(false);//Derek is mad cute
chain_drive(5,100);
chain_turn(-55,127); // Turn to face corner blocks
chain_drive(18.5, 127);
// Loader.set_value(true);
chain_drive(8, 127); // Drive forward to corner blocks
turn(-91,75); // Turn to face 2nd set of corner blocks
chain_drive(6,127); // Drive to 2nd set of corner blocks
// Loader.set_value(false);
chain_drive(31, 127);
// Loader.set_value(true);
chain_drive(6,120);
chain_turn(-134.8,100); // Turn to face center goal
chassis.pid_drive_exit_condition_set(40_ms, 5_in, 80_ms, 7_in, 150_ms, 150_ms);
chassis.pid_drive_set(-19, 50, true);
chassis.pid_wait_until(-17); // slow into contact
pros::delay(100);
// Deflector.set_value(true);

chassis.pid_drive_exit_condition_set(40_ms, 2.5_in, 80_ms, 4_in, 150_ms, 150_ms);
chain_turn(-135, 127);
pros::delay(650); 
pros::delay(100);
drive(53.7, 127);
chain_turn(180, 127);//Derek is mad cute
// Deflector.set_value(false);
chain_drive(7, 100);
max_drive(6, 80);
pros::delay(410); // Wait to intake blocks from Loader
chain_drive(-26,100); // Drive back to score on long goal
pros::delay(100);
pros::delay(1500);
}



void drive_skills(){
chassis.drive_brake_set(pros::E_MOTOR_BRAKE_COAST); 
}

// SKILLS ------------------------------------------------------------------------------------------------------------
void skills(){
 chassis.odom_xyt_set(-55_in, 12_in, 90_deg); // Initialize the odometry with specified parameters
//  Descore.set_value(true);
 chain_drive(11, 10,100);
 chain_turn(45, 100);
 chain_drive(2,60);
 drive(5, 20);
 pros::delay(100);
 chain_turn(-68, 80);
 chain_drive(-15.5, 100);
 chain_turn(-45, 100);
//  Deflector.set_value(true);
 chain_drive(-.5, 100);
 pros::delay(1200); 
//  Loader.set_value(true);
 drive(54, 127);
 chain_turn(-90, 120);
//  Deflector.set_value(false);
 chain_drive(11, 100);
 pros::delay(1200); // Wait to intake blocks from Loader
 chain_drive(-.5, 50);
 chain_drive(.5, 50);
 pros::delay(300);
 chain_drive(-11, 127);
 chain_turn(-135, 127);//Derek is mad cute
 chain_drive(-9, 100);
//  Loader.set_value(false);
 chain_turn(-89, 90);
 chain_drive(-64.5, 127);
 chain_turn(180, 100);
 drive(8.5, 127);
 turn(90, 100); 
 chain_drive(-8, 100);
//  Loader.set_value(true);//Derek is mad cute
 pros::delay(1400); // Wait to score in long goal
 chain_turn(92, 100);
 chain_drive(27, 100); //drive into match loader
 pros::delay(900);
 chain_drive(-.5, 50);
 chain_drive(.5, 50);
 pros::delay(300);//Derek is mad cute
 chain_turn(93, 100);
 chain_drive(-25, 100);
//  Loader.set_value(false);
 pros::delay(1000); // Wait to score on long goal 
 pros::delay(850);
 chassis.pid_targets_reset(); //Derek is mad cute // Resets PID targets to 0
 chassis.drive_imu_reset(); // Reset gyro position to 0
 chassis.drive_sensor_reset(); // Reset drive sensors to 0
 chassis.odom_xyt_set(0_in, 0_in, 90_deg); // Set the current position
 chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD); // Hold mode for consistency
 chassis.pid_turn_exit_condition_set(100_ms, 1_deg, 200_ms, 3_deg, 300_ms, 300_ms);
 chassis.pid_drive_exit_condition_set(40_ms, .5_in, 80_ms, 1_in, 150_ms, 150_ms);
 drive(14, 127);
 turn(178, 127);
 drive(44.5, 127);
 turn(87, 127);
 chassis.pid_turn_exit_condition_set(40_ms, 4_deg, 80_ms, 7_deg, 150_ms, 150_ms);
 chassis.pid_drive_exit_condition_set(40_ms, 2.5_in, 80_ms, 4_in, 150_ms, 150_ms);
max_drive(30 , 80); // Drive into park zone
 max_drive(6, 60);
 max_drive(-0.4, 60);
 pros::delay(400);
 max_drive(14, 30);
 max_drive(-0.6, 60);
 max_drive(4, 30);
 pros::delay(400);
 max_drive(-20, 100); //Drive out of park zone
 pros::delay(100);
 max_drive(2, 50);
 pros::delay(200);
 chain_drive(-5, 127);
 chain_turn(45, 100);
 chain_drive(-19, 120);
 chain_turn(135, 120);
 chain_drive(-11.2, 120); // Drive to middle goal 
 pros::delay(100);
 pros::delay(100);
//  Deflector.set_value(true);
 drive(1, 60);
 pros::delay(1000);
 pros::delay(1200);
 drive(2.5, 60);
 pros::delay(400);
 drive(-3.5, 50);
 pros::delay(300);
//  Loader.set_value(true);
 drive(48.5, 127);
 chain_turn(88, 120);
//  Deflector.set_value(false);
 chain_drive(15, 100);
 pros::delay(1200); // Wait to intake blocks from Loader
 chain_drive(-.5, 50);
 chain_drive(.5, 50);
 pros::delay(300);
 chain_drive(-11, 127);
 chain_turn(45, 127);//Derek is mad cute
 chain_drive(-11.2, 100);
//  Loader.set_value(false);
 chain_turn(88, 90);
 chain_drive(-64.5, 127);
 chain_turn(0, 100);
 drive(9.7, 127);
 turn(-90, 100); 
 chain_drive(-11, 100);
//  Loader.set_value(true);//Derek is mad cute

 pros::delay(1400); // Wait to score in long goal
 chain_turn(-91, 100);
 chain_drive(27, 90); //drive into match loader
 pros::delay(900);
 chain_drive(-.5, 50);
 chain_drive(.5, 50);
 pros::delay(300);//Derek is mad cute
 chain_turn(-89, 100);
 chain_drive(-25, 100);
//  Loader.set_value(false);
 pros::delay(1000); // Wait to score on long goal 
 pros::delay(700);
 chassis.pid_targets_reset(); //Derek is mad cute // Resets PID targets to 0
 chassis.drive_imu_reset(); // Reset gyro position to 0
 chassis.drive_sensor_reset(); // Reset drive sensors to 0
 chassis.odom_xyt_set(0_in, 0_in, 90_deg); // Set the current position
 chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD); // Hold mode for consistency
 chassis.pid_turn_exit_condition_set(100_ms, 1_deg, 200_ms, 3_deg, 300_ms, 300_ms);
 chassis.pid_drive_exit_condition_set(40_ms, 1.5_in, 80_ms, 2_in, 150_ms, 150_ms);
 drive(14, 127);
 turn(178, 127);
 drive(44.5, 127);
 turn(87, 127);
 chassis.pid_turn_exit_condition_set(40_ms, 4_deg, 80_ms, 7_deg, 150_ms, 150_ms);
 chassis.pid_drive_exit_condition_set(40_ms, 2.5_in, 80_ms, 4_in, 150_ms, 150_ms);
 max_drive(30 , 80); // Drive into park zone
 max_drive(5, 60);
 max_drive(-.4, 60);
 pros::delay(400);
 max_drive(14, 30);
 max_drive(-.6, 60);
 max_drive(4, 30);
 pros::delay(400);
 }

//for skills swing
// Descore.set_value(true);
// chain_lswing(178, 175, 57, 90);
// chain_turn(178, 100);
// intake(127);
// max_drive(15, 100);
// max_drive(51, 80);
// max_drive(15, 100);
// turn(180, 120);
// max_drive(-17, 50);
// chain_lswing(270, 265, 0, 100);

 // For Drive:
 // The first parameter is target inches
 // The second parameter is max speed the robot will drive at
 // The third parameter is a boolean (true or false) for enabling/disabling a slew at the start of drive motions
 // for slew, only enable it when the drive distance is greater than the slew distance + a few inches

 // For Turn:
 // The first parameter is the target in degrees
 // The second parameter is max speed the robot will drive at

///
// Combining Turn + Drive
///
void drive_and_turn() {
 chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
 chassis.pid_wait();

 chassis.pid_turn_set(45_deg, TURN_SPEED);
 chassis.pid_wait();

 chassis.pid_turn_set(-45_deg, TURN_SPEED);
 chassis.pid_wait();

 chassis.pid_turn_set(0_deg, TURN_SPEED);
 chassis.pid_wait();

 chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
 chassis.pid_wait();
}

///
// Wait Until and Changing Max Speed
///
void wait_until_change_speed() {
 // pid_wait_until will wait until the robot gets to a desired position

 // When the robot gets to 6 inches slowly, the robot will travel the remaining distance at full speed
 chassis.pid_drive_set(24_in, 30, true);
 chassis.pid_wait_until(6_in);
 chassis.pid_speed_max_set(DRIVE_SPEED); // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
 chassis.pid_wait();

 chassis.pid_turn_set(45_deg, TURN_SPEED);
 chassis.pid_wait();

 chassis.pid_turn_set(-45_deg, TURN_SPEED);
 chassis.pid_wait();

 chassis.pid_turn_set(0_deg, TURN_SPEED);
 chassis.pid_wait();

 // When the robot gets to -6 inches slowly, the robot will travel the remaining distance at full speed
 chassis.pid_drive_set(-24_in, 30, true);
 chassis.pid_wait_until(-6_in);
 chassis.pid_speed_max_set(DRIVE_SPEED); // After driving 6 inches at 30 speed, the robot will go the remaining distance at DRIVE_SPEED
 chassis.pid_wait();
}

///
// Swing Example
///
void swing_example() {
 // The first parameter is ez::LEFT_SWING or ez::RIGHT_SWING
 // The second parameter is the target in degrees
 // The third parameter is the speed of the moving side of the drive
 // The fourth parameter is the speed of the still side of the drive, this allows for wider arcs

 chassis.pid_swing_set(ez::LEFT_SWING, 45_deg, SWING_SPEED, 45);
 chassis.pid_wait();

 chassis.pid_swing_set(ez::RIGHT_SWING, 0_deg, SWING_SPEED, 45);
 chassis.pid_wait();

 chassis.pid_swing_set(ez::RIGHT_SWING, 45_deg, SWING_SPEED, 45);
 chassis.pid_wait();

 chassis.pid_swing_set(ez::LEFT_SWING, 0_deg, SWING_SPEED, 45);
 chassis.pid_wait();
}

///
// Motion Chaining
///
void motion_chaining() {
 // Motion chaining is where motions all try to blend together instead of individual movements.
 // This works by exiting while the robot is still moving a little bit.
 // To use this, replace pid_wait with pid_wait_quick_chain.
 chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
 chassis.pid_wait();

 chassis.pid_turn_set(45_deg, TURN_SPEED);
 chassis.pid_wait_quick_chain();

 chassis.pid_turn_set(-45_deg, TURN_SPEED);
 chassis.pid_wait_quick_chain();

 chassis.pid_turn_set(0_deg, TURN_SPEED);
 chassis.pid_wait();

 // Your final motion should still be a normal pid_wait
 chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
 chassis.pid_wait();
}

///
// Auto that tests everything
///
void combining_movements() {
 chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
 chassis.pid_wait();

 chassis.pid_turn_set(45_deg, TURN_SPEED);
 chassis.pid_wait();

 chassis.pid_swing_set(ez::RIGHT_SWING, -45_deg, SWING_SPEED, 45);
 chassis.pid_wait();

 chassis.pid_turn_set(0_deg, TURN_SPEED);
 chassis.pid_wait();

 chassis.pid_drive_set(-24_in, DRIVE_SPEED, true);
 chassis.pid_wait();
}



// If there is no interference, the robot will drive forward and turn 90 degrees.
// If interfered, the robot will drive forward and then attempt to drive backward.
void interfered_example() {
 chassis.pid_drive_set(24_in, DRIVE_SPEED, true);
 chassis.pid_wait();

 if (chassis.interfered) {
 tug(3);
 return;
 }

 chassis.pid_turn_set(90_deg, TURN_SPEED);
 chassis.pid_wait();
}

///
// Odom Drive PID
///
void odom_drive_example() {
 // This works the same as pid_drive_set, but it uses odom instead!
 // You can replace pid_drive_set with pid_odom_set and your robot will
 // have better error correction.

 chassis.pid_odom_set(24_in, DRIVE_SPEED, true);
 chassis.pid_wait();

 chassis.pid_odom_set(-12_in, DRIVE_SPEED);
 chassis.pid_wait();

 chassis.pid_odom_set(-12_in, DRIVE_SPEED);
 chassis.pid_wait();
}

///
// Odom Pure Pursuit
///
void odom_pure_pursuit_example() {
 // Drive to 0, 30 and pass through 6, 10 and 0, 20 on the way, with slew
 chassis.pid_odom_set({{{6_in, 10_in}, fwd, DRIVE_SPEED},
 {{9_in, 20_in}, rev, DRIVE_SPEED},
 {{0_in, 30_in}, fwd, DRIVE_SPEED}},
 true);
 chassis.pid_wait();

 // Drive to 0, 0 backwards
 chassis.pid_odom_set({{0_in, 0_in}, rev, DRIVE_SPEED},
 true);
 chassis.pid_wait();

 chassis.pid_odom_set({{{0_in, 0_in}, fwd, DRIVE_SPEED},
 {{0_in, 24_in}, fwd, DRIVE_SPEED}},
 true);
}

///
// Odom Pure Pursuit Wait Until
///
void odom_pure_pursuit_wait_until_example() {
 chassis.pid_odom_set({{{0_in, 24_in}, fwd, DRIVE_SPEED},
 {{12_in, 24_in, 90_deg}, fwd, DRIVE_SPEED},
 {{24_in, 36_in, -20_deg}, fwd, DRIVE_SPEED}},
 true);
 chassis.pid_wait_until_index(1); // Waits until the robot passes 12, 24
 //intake(127); // Set your intake to start moving once it passes through the second point in the index
 chassis.pid_wait();
 //intake(0); // Turn the intake off
}

///
// Odom Boomerang
///
void odom_boomerang_example() {
 chassis.pid_odom_set({{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
 true);
 chassis.pid_wait();

 chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
 true);
 chassis.pid_wait();
}

///
// Odom Boomerang Injected Pure Pursuit
///
void odom_boomerang_injected_pure_pursuit_example() {
 chassis.pid_odom_set({{{0_in, 24_in, 45_deg}, fwd, DRIVE_SPEED},
 {{12_in, 24_in}, fwd, DRIVE_SPEED},
 {{24_in, 24_in}, fwd, DRIVE_SPEED}},
 true);
 chassis.pid_wait();

 chassis.pid_odom_set({{0_in, 0_in, 0_deg}, rev, DRIVE_SPEED},
 true);
 chassis.pid_wait();
}

///
// Calculate the offsets of your tracking wheels
///
void measure_offsets() {
 // Number of times to test
 int iterations = 10;

 // Our final offsets
 double l_offset = 0.0, r_offset = 0.0, b_offset = 0.0, f_offset = 0.0;

 // Reset all trackers if they exist
 if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->reset();
 if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->reset();
 if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->reset();
 if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->reset();
 
 for (int i = 0; i < iterations; i++) {
 // Reset pid targets and get ready for running an auton
 chassis.pid_targets_reset();
 chassis.drive_imu_reset();
 chassis.drive_sensor_reset();
 chassis.drive_brake_set(pros::E_MOTOR_BRAKE_HOLD);
 chassis.odom_xyt_set(0_in, 0_in, 0_deg);
 double imu_start = chassis.odom_theta_get();
 double target = i % 2 == 0 ? 90 : 270; // Switch the turn target every run from 270 to 90

 // Turn to target at half power
 chassis.pid_turn_set(target, 63, ez::raw);
 chassis.pid_wait();
 pros::delay(250);

 // Calculate delta in angle
 double t_delta = util::to_rad(fabs(util::wrap_angle(chassis.odom_theta_get() - imu_start)));

 // Calculate delta in sensor values that exist
 double l_delta = chassis.odom_tracker_left != nullptr ? chassis.odom_tracker_left->get() : 0.0;
 double r_delta = chassis.odom_tracker_right != nullptr ? chassis.odom_tracker_right->get() : 0.0;
 double b_delta = chassis.odom_tracker_back != nullptr ? chassis.odom_tracker_back->get() : 0.0;
 double f_delta = chassis.odom_tracker_front != nullptr ? chassis.odom_tracker_front->get() : 0.0;

 // Calculate the radius that the robot traveled
 l_offset += l_delta / t_delta;
 r_offset += r_delta / t_delta;
 b_offset += b_delta / t_delta;
 f_offset += f_delta / t_delta;
 }

 // Average all offsets
 l_offset /= iterations;
 r_offset /= iterations;
 b_offset /= iterations;
 f_offset /= iterations;

 // Set new offsets to trackers that exist
 if (chassis.odom_tracker_left != nullptr) chassis.odom_tracker_left->distance_to_center_set(l_offset);
 if (chassis.odom_tracker_right != nullptr) chassis.odom_tracker_right->distance_to_center_set(r_offset);
 if (chassis.odom_tracker_back != nullptr) chassis.odom_tracker_back->distance_to_center_set(b_offset);
 if (chassis.odom_tracker_front != nullptr) chassis.odom_tracker_front->distance_to_center_set(f_offset);
}
