#pragma once

#include "controls.hpp"

namespace Controller {
    void update(const Controls& driver);
    void execute();

    extern bool auton;

    extern int drive;
    extern int steer;

    extern bool liftUp;
    extern bool liftDown;

    extern bool openClaw;

    extern bool toggleOn;
}