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
    chassis.moveToPoint(20, 20, 5000);
}

void opcontrol() {
    while (true) {
        if (Controller::auton) {autonomous();}
        Controller::update(john);
        Controller::execute();
        pros::delay(20);
    }
}