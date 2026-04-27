#include "components.hpp"

//lift brake mode is HOLD, in main.cpp intialize()

int queue = 0; //add rapid fire system
void Components::toggle(bool once) {
    if (once) {Hardware::toggle.move_relative(-360, 200);}
}
