#include "controls.hpp"
#include "pros/misc.h"

Controls base {};
Controls john {
    .liftUp = pros::E_CONTROLLER_DIGITAL_B,
    .liftDown = pros::E_CONTROLLER_DIGITAL_DOWN,
    .openClaw = pros::E_CONTROLLER_DIGITAL_R2,
    .toggle = pros::E_CONTROLLER_DIGITAL_L2
};