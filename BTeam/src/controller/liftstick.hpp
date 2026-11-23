#pragma once

#include "controls.hpp"

// Optional raw lift control on the right stick's vertical axis.
// It skips the button smoothing. The top and bottom limits still stop it.
// The lift buttons keep working and win if both are used.
//
// To turn it off: set ENABLED to false.
// To remove it: delete liftstick.hpp and liftstick.cc, then follow the
// LIFT STICK comments in controller.cc.
namespace LiftStick {
inline constexpr bool ENABLED = true;

struct Output {
  int liftPower; // 0 when the stick isn't moving the lift
  int steer;     // steering with thumb drift removed while lifting
};

// Call once every driver loop
Output read(const Controls &driver);
} // namespace LiftStick
