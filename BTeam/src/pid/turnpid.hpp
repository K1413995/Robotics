#pragma once

// Turn PID for macros, using the same TURN values as LemLib's turns.
// LemLib's turn commands can't change target mid-move, so macros use this.
namespace TurnPid {
struct Gains {
  float kP;
  float kI;
  float kD;
};

// The tuner calls this so macros use the tuned values too
void setGains(Gains next);

// Call when a move starts or its target changes
void restart(double error);
// Call every 10 ms. Returns motor power, positive turns right
double update(double error);
} // namespace TurnPid
