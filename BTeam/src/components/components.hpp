#pragma once

#include "../controller/controller.hpp"
#include "../hardware/hardware.hpp"

namespace Components {
void startLift();
void lift(int);
void liftTo(double angle);
void liftRaw(int power);
void waitForLift(int timeoutMs = 2000);
bool liftSettled();

void clawIntake(bool);
void clawOpen(bool);
} // namespace Components
