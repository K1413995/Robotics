#pragma once

#include "pros/misc.h"

struct Controls {
    pros::controller_digital_e_t auton = pros::E_CONTROLLER_DIGITAL_A;

    pros::controller_analog_e_t leftY = pros::E_CONTROLLER_ANALOG_LEFT_Y;
    pros::controller_analog_e_t rightX = pros::E_CONTROLLER_ANALOG_RIGHT_X;

    pros::controller_digital_e_t liftUp = pros::E_CONTROLLER_DIGITAL_L1;
    pros::controller_digital_e_t liftDown = pros::E_CONTROLLER_DIGITAL_L2;

    pros::controller_digital_e_t openClaw = pros::E_CONTROLLER_DIGITAL_X;

    pros::controller_digital_e_t toggle = pros::E_CONTROLLER_DIGITAL_Y;
};

// Driver configurations
extern Controls base;
extern Controls john;