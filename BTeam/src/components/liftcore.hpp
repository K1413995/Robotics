#pragma once

#include <cstdint>

// Lift basics shared by the driver smoothing and the target PID
namespace LiftCore {
// Height is degrees above the bottom limit
double angleToHeight(double angle);
double range();
double roomToLimit(bool goingUp, double height);

// S-curve between 0 and 1
double smoothStep(double t);

// Raw motor power. 0 brakes once so HOLD keeps the spot
void setPower(int power);
// Speed at the lift, using the motor's built-in velocity control
void setSpeed(double liftSpeed);
// The speed last sent with setSpeed, 0 if the motors aren't in speed control
double commandedSpeed();

// True once the lift has barely moved for stallLimit milliseconds
bool isStalled(double height, std::uint32_t stallLimit);
void resetStall();
} // namespace LiftCore
