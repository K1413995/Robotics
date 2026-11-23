#pragma once

#include "../controller/controller.hpp"
#include "../hardware/hardware.hpp"

extern lemlib::Chassis chassis;
extern lemlib::Drivetrain drivetrain;

namespace Components {
void startLift();
void lift(int);
void liftTo(double angle);
void liftBy(double degrees);
void liftRaw(int power);
void waitForLift(int timeoutMs = 2000);
bool liftSettled();

void clawIntake(bool);
void clawOpen(bool);
} // namespace Components
