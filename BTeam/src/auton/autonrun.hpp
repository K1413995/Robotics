#pragma once

#include <cstdint>

// Runs the auton on its own task in the test code, so it can be canceled
namespace AutonRun {
void start();
void cancel();
// Call every loop, keeps stopping moves until a canceled auton has exited
void update();
bool running();
// Length of the current or last run, restarts on every start
std::uint32_t elapsedMs();

// For auton.cc: true means stop and return
bool canceled();
// A delay that ends early if canceled. Returns false if canceled
bool wait(std::uint32_t ms);
} // namespace AutonRun
