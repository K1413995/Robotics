#pragma once

#include "../hardware/hardware.hpp"
#include "../controller/controller.hpp"

namespace Components {
    void lift(int);

    void clawIntake(bool);
    void clawOpen(bool);

    void toggle(bool);
}