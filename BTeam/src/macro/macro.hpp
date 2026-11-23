#pragma once

namespace Macro {
// Toggle sequence: lift to the angle, back up, raise the lift, nudge forward.
// Call every loop. Returns true while the macro is controlling the robot.
// driverInput cancels it, unless that input was already held when it started.
bool toggle(bool pressed, bool driverInput);

// Stops the macro right away and holds the lift where it is
void cancel();
} // namespace Macro
