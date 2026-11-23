#include "main.h"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include "auton/auton.hpp"
#include "components/components.hpp"
#include "controller/controller.hpp"
#include "controller/controls.hpp"
#include "hardware/hardware.hpp"
#include "interface/interface.hpp"
#include "interface/settings.hpp"
#include "interface/testmode.hpp"
#include "macro/macro.hpp"
#include "pid/pidtune.hpp"

// true uploads the TEST code: controller modes, auton on A, and the PID tuner
// false uploads the COMP code: normal driver control with no menus or modes
// Plugged into a field or competition switch, the test code also acts like comp
constexpr bool TEST_MODE = true;

// The side the COMP code runs, since it has no menu to pick one
constexpr Settings::Side COMP_SIDE = Settings::Side::Left;

void on_center_button() {}

void initialize() {
  Interface::start();
  if (TEST_MODE) {
    PidTune::start();
  }
  Hardware::liftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
  Components::startLift();
  Macro::startMovement();
  chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
  chassis.calibrate();
  Hardware::claw.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  Settings::setSide(COMP_SIDE);
  setStartPose();
}

void disabled() {}

void competition_initialize() {}

void autonomous() { auton(); }

void opcontrol() {
  const bool testing = TEST_MODE && !pros::competition::is_connected();
  if (testing) {
    TestMode::begin();
  }

  while (true) {
    Controls::update(john);
    if (testing) {
      TestMode::update(john);
    } else {
      Controller::execute(john);
    }
    pros::delay(20);
  }
}
