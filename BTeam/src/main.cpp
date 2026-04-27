#include "main.h"
#include "pros/rtos.hpp"

#include "lemlib/drivetrain.hpp"
#include "hardware/hardware.hpp"
#include "controller/controller.hpp"
#include "controller/controls.hpp"
#include "components/components.hpp"

void on_center_button() {}

void initialize() {
    Hardware::liftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD);
    chassis.setBrakeMode(pros::E_MOTOR_BRAKE_BRAKE);
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    // chassis.setPose(0, 0, 0);
    // Components::clawUp(false);
    // chassis.moveToPoint(0, 60, 5000);
    // Hardware::liftMotors.move_relative(100, 400);
    // Components::openClaw(true);
    // Components::clawUp(false);
    // chassis.arcade(90, 0);
    // pros::delay(3000);
    // chassis.arcade(0, 0);
    // chassis.arcade(-60, 0);
    // pros::delay(250);
    // chassis.arcade(0, 0);
    // Hardware::controller.get_digital('A');
}

void opcontrol() {
    Components::clawUp(false);
    while (true) {
        if (Controller::auton) {autonomous();}
        Controller::update(john);
        Controller::execute();
        pros::delay(20);
    }
}