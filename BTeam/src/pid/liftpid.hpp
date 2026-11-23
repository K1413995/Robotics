#pragma once

// Normal target PID for the lift: give it a height and it goes there and holds
namespace LiftPid {
struct Gains {
  float kP;
  float kI;
  float kD;
};

// The values in memory, the PID tuner uses these
Gains gains();
void setGains(Gains next);

// Heights are degrees above the bottom limit
void setTarget(double target, double height);
void clear();

// Call every loop. Returns true on the loop the move finishes
bool run(double height);
} // namespace LiftPid
