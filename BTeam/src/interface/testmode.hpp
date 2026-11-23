#pragma once

#include "../controller/controls.hpp"

// The test code's modes, picked by double tapping on the controller
//   A A: auto mode. One tap of A starts or stops the auton,
//        and left or right picks the side while it is stopped
//   X X: driver control
//   Y Y: PID tuner
namespace TestMode {
// Call once at the start of opcontrol
void begin();
// Call every loop in place of normal driver control
void update(const Controls &driver);
} // namespace TestMode
