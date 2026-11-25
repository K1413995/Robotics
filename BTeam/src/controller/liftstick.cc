// LIFT STICK
// The right stick's vertical axis drives the lift motors directly.
// No ramp or curve, so the driver sets the speed. The only slowing is near
// the bottom on the way down, done in components/lift.cc
// Steering is on the same stick, so each axis is protected from the other.
#include "liftstick.hpp"

#include <algorithm>
#include <cstdlib>

namespace LiftStick {
namespace {

// DEADBAND, out of 127
// The lift starts past this, and only if the stick is more up/down than sideways
constexpr int START_STICK = 45;
// Once moving, it stops below this. The gap prevents flickering at the edge
constexpr int STOP_STICK = 35;

// Motor power out of 127, in a straight line across the stick's travel
constexpr int MIN_POWER = 1;  // right past the deadband
constexpr int MAX_POWER = 95; // stick all the way (95 is 75%)

// While lifting, steering ignores this much sideways thumb drift, out of 127
constexpr int LIFT_STEER_DEADBAND = 35;

bool engaged = false;

int stickPower(int stick) {
  const int firstStick = START_STICK + 1;
  const int travel = std::max(std::abs(stick) - firstStick, 0);
  const int power =
      MIN_POWER + travel * (MAX_POWER - MIN_POWER) / (127 - firstStick);
  return stick > 0 ? power : -power;
}

// Removes the drift, then rescales so full steering is still available
int steerWhileLifting(int steer) {
  const int size = std::abs(steer);
  if (size <= LIFT_STEER_DEADBAND) {
    return 0;
  }
  const int scaled =
      (size - LIFT_STEER_DEADBAND) * 127 / (127 - LIFT_STEER_DEADBAND);
  return steer > 0 ? scaled : -scaled;
}

} // namespace

Output read(const Controls &driver) {
  const int y = driver.rightY.value();
  const int x = driver.rightX.value();
  if (!ENABLED) {
    return {0, x};
  }

  if (engaged && !driver.rightY.past(STOP_STICK)) {
    engaged = false;
  } else if (!engaged && driver.rightY.past(START_STICK) &&
             std::abs(y) > std::abs(x)) {
    engaged = true;
  }
  if (!engaged) {
    return {0, x};
  }
  return {stickPower(y), steerWhileLifting(x)};
}

} // namespace LiftStick
