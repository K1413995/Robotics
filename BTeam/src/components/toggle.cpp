#include "components.hpp"

//lift brake mode is HOLD, in main.cpp intialize()

void Components::toggle(bool on) {
    if (on) {Hardware::toggle.move(60);}
    else {Hardware::toggle.brake();}
}
