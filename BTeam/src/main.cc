#include "main.h"
#include "pros/rtos.hpp"

#include "auton/auton.hpp"
#include "components/components.hpp"
#include "controller/controller.hpp"
#include "controller/controls.hpp"
#include "hardware/hardware.hpp"
#include "interface/interface.hpp"
#include "interface/settings.hpp"
#include "lemlib/drivetrain.hpp"
#include "macro/macro.hpp"
#include "pidtune/pidtune.hpp"

void on_center_button() {}

void initialize() {
  Interface::start();
  PidTune::start();
  Hardware::liftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  Components::startLift();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.calibrate();
  Hardware::claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  setStartPose();
}

void disabled() {}

void competition_initialize() {}

void autonomous() { auton(); }

void opcontrol() {
  Settings::open();
  while (true) {
    if (john.auton.newPress()) {
      Macro::cancel();
      autonomous();
    }
    Controls::update(john);
    // The menu and PID tuner take over the controls while they are open
    if (!Settings::update(john) && !PidTune::update(john)) {
      Controller::execute(john);
    }
    pros::delay(20);
  }
}
