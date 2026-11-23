#pragma once

#include <cstdint>

// PID VALUES
// The defaults for every PID on the robot. The tuner only changes them in
// memory, so copy values you like into this file.
namespace PidConfig {

struct Motion {
  float kP;                // proportional gain
  float kI;                // integral gain
  float kD;                // derivative gain
  float windup;            // anti windup
  float smallError;        // small error range, inches or degrees
  float smallErrorTimeout; // small error range timeout, in milliseconds
  float largeError;        // large error range, inches or degrees
  float largeErrorTimeout; // large error range timeout, in milliseconds
  float slew;              // maximum acceleration
};

// Forward and back (NEED TO UNDERSTAND)
constexpr Motion DRIVE{10, 0, 3, 3, 1, 100, 3, 500, 20};

// Turning (NEED TO UNDERSTAND)
constexpr Motion TURN{2, 0, 10, 3, 1, 100, 3, 500, 0};

// Lift, used by liftTo and liftBy, and to hold position
namespace Lift {
// kP moves it, kD stops overshoot, kI adds power for heavier stacks
constexpr float KP = 4;
constexpr float KI = 0.2f;
constexpr float KD = 15;
// kI only builds up within this many degrees of the target
constexpr float INTEGRAL_RANGE = 15;
// The PID gets longer than the driver before it counts as blocked,
// so kI has time to build enough power for a heavy stack
constexpr std::uint32_t PID_STALL_TIME = 400;
// Strongest power the PID can use, out of 127
constexpr int PID_MAX_POWER = 95;
// Weakest power that still moves the lift, only used while it is moving
constexpr int MIN_UP_POWER = 25;
constexpr int MIN_DOWN_POWER = 10;
// Most the PID's motor power can change every loop
constexpr double POWER_STEP = 10;
// Within this many degrees of the target, HOLD brake takes over
constexpr double TOLERANCE = 1.5;
// While braked, the PID wakes up if the lift drifts farther than this
constexpr double DRIFT_LIMIT = 4;
} // namespace Lift

} // namespace PidConfig
