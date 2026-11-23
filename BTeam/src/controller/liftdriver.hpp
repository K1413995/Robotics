#pragma once

// Smooths the driver's lift buttons. There is no target and no PID here
namespace LiftDriver {
// Call every loop while a button is held. Positive power is up
void run(int power, double height);

// Call once when the button is let go
void beginRelease();
bool releasing();
// Returns true while the release is still fading out
bool runRelease(double height);

// Forget the current press and release
void reset();
} // namespace LiftDriver
