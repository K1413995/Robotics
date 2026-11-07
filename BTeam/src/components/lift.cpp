#include "components.hpp"

//lift brake mode is HOLD, in main.cpp intialize()

void Components::lift(int voltage) {
    if (voltage == 0) {Hardware::liftMotors.brake();}
    else {Hardware::liftMotors.move(voltage);}
}