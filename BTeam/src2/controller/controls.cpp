#include "controls.hpp"
#include "pros/misc.h"

Controls base {};
Controls john {
    .liftUp = pros::E_CONTROLLER_DIGITAL_R2,
    .liftDown = pros::E_CONTROLLER_DIGITAL_R1
};