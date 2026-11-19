#include "main.h"
#include "pros/rtos.hpp"

#include "lemlib/drivetrain.hpp"
#include "auton/auton.hpp"
#include "hardware/hardware.hpp"
#include "controller/controller.hpp"
#include "controller/controls.hpp"

void on_center_button() {}

void initialize() {
    Hardware::liftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
    chassis.calibrate();
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    auton();
}

void opcontrol() {
    while (true) {
        if (john.auton.newPress()) {autonomous();}
        Controls::update(john);
        Controller::execute(john);
        pros::delay(20);
    }
}