#pragma once

namespace Macro {
// TOGGLE (toggle.cc)
// Toggle sequence: lift to the angle, back up, raise the lift, nudge forward.
// Call every loop. Returns true while the macro is controlling the robot.
// driverInput cancels it, unless that input was already held when it started.
bool toggle(bool pressed, bool driverInput);
// Stops the toggle right away and holds the lift where it is
void cancelToggle();

// MOVEMENT (movement.cc)
// Starts the task that runs the turns, call once in initialize()
void startMovement();
// Left and right turn to the next cardinal direction. Extra presses during a
// turn add or remove 90 degrees. Call every loop with new presses only.
// Returns true while the macro is controlling the drive.
// stickInput cancels it, unless the stick was already held when it started.
bool turn(bool left, bool right, bool stickInput);
// Stops the turn right away
void cancelTurn();

// Stops every macro
inline void cancel() {
  cancelToggle();
  cancelTurn();
}
} // namespace Macro
