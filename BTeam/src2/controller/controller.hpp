#pragma once

#include "../components/components.hpp"
#include "controls.hpp"

namespace Controller {
    void update(const Controls& driver);
    void act();

    extern int leftY;
    extern int rightX;

    extern bool liftUp;
    extern bool LiftDown;

    extern bool openClaw;

    extern bool toggle;
}