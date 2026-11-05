#include "main.h"
#include "pros/rtos.hpp"

#include "lemlib/drivetrain.hpp"
#include "hardware/hardware.hpp"
#include "controller/controller.hpp"
#include "controller/controls.hpp"

void on_center_button() {}

void initialize() {
    Hardware::liftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.setPose(0, 0, 0);
    //chassis.arcade(0, 50);
}

void opcontrol() {
    while (true) {
        Controller::update(john);
        Controller::act();
        pros::delay(20);
    }
}