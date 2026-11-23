#pragma once

#include "../controller/controls.hpp"

namespace PidTune {
// Sets up the tuner's brain screen, call once in initialize()
void start();

// Called by the test code when entering and leaving PID mode
void open();
void close();

// Call every loop while in PID mode
void update(const Controls &driver);
} // namespace PidTune
