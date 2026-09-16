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

inline pros::Rotation Rotation(21);

inline void set_Lift(int input) {
  LiftLeft.move(input);
  LiftRight.move(input);
}

// P, I, D, Start I
inline ez::PID lift_controller{3.6, 0, 7, 0, "Lift"};

inline double get_Lift_position() {
  return Rotation.get_position() / 100.0;
}



inline void lift_wait() {
  while (lift_controller.exit_condition({LiftLeft, LiftRight}, true) == ez::RUNNING) {
    pros::delay(ez::util::DELAY_TIME);
  }
}

inline bool lift_pid_enabled = false;

inline void set_lift_pos(double target) {
  lift_controller.target_set(target);
  lift_controller.timers_reset();
  lift_pid_enabled = true;
}

inline void stop_lift_pid() {
  lift_pid_enabled = false;
  set_Lift(0);
}

inline void lift_pid_task(void*) {
  while (true) {
    if (lift_pid_enabled) {
      double position = get_Lift_position();
      double output = lift_controller.compute(position);

      set_Lift(static_cast<int>(output));
    }

    pros::delay(10);
  }
}

inline void start_lift_pid_task() {
  // "static" ensures only one lift task is ever created.
  static pros::Task lift_task(
      lift_pid_task,
      nullptr,
      "Lift PID"
  );
}
//--------------------------------------------------------------



inline pros::Motor Claw(-8, pros::MotorGearset::green);
inline pros::Motor Intake(9, pros::MotorGearset::green);