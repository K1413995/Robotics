#pragma once

#include "../controller/controls.hpp"

namespace PidTune {
// Sets up the tuner's brain screen, call once in initialize()
void start();

// Call every loop. Hold X + Y for 1 second to open the tuner.
// Returns true while the tuner is controlling the robot.
bool update(const Controls &driver);
} // namespace PidTune
