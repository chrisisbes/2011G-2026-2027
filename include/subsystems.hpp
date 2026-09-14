#pragma once

#include "EZ-Template/api.hpp"
#include "api.h"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/rotation.hpp"

extern Drive chassis;

// Your motors, sensors, etc. should go here.  Below are examples
//Lift------------------------------------------------------
inline pros::Motor LiftLeft(-17, pros::MotorGearset::blue);
inline pros::Motor LiftRight(18, pros::MotorGearset::blue);

inline pros::Rotation Rotation(-21);

inline void set_Lift(int input) {
  LiftLeft.move(input);
  LiftRight.move(input);
}

// P, I, D, Start I
inline ez::PID lift_controller{0.1, 0, 0, 0, "Lift"};

inline double get_Lift_position() {
  return Rotation.get_position() / 100.0;
}

inline void set_lift_pos(double target) {
  lift_controller.target_set(target);
}

inline void lift_wait() {
  while (lift_controller.exit_condition({LiftLeft, LiftRight}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}
inline bool manualLift = false;
//--------------------------------------------------------------



inline pros::Motor Claw(-8, pros::MotorGearset::green);
inline pros::Motor Intake(9, pros::MotorGearset::green);