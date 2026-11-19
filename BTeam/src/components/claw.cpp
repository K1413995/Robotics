#include "components.hpp"

namespace Components {
    void clawIntake(bool in) {
        if (in) {Hardware::claw.move(60);} 
        else {Hardware::claw.brake();}
    }

    void clawOpen(bool open) {
        if (open) {
            Hardware::claw.move_relative(-120, 60);
        }
    }
}