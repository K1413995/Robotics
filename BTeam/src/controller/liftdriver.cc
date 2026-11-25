// LIFT DRIVER SMOOTHING
// The code can't predict the driver, so it shapes speed from what it knows:
//   How long the button is held: taps stay slow, holds build to full speed
//   Where the lift is: it slows down before the top and bottom limits
//   How fast it is going: faster lifts get a longer fade when let go
// Every speed change is interpolated so the bands don't start bouncing
#include "liftdriver.hpp"
#include "../components/liftconfig.hpp"
#include "../components/liftcore.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace LiftDriver {
namespace {

using namespace LiftConfig;
using LiftCore::smoothStep;

int holdDirection = 0;
std::uint32_t holdStart = 0;
double carriedSpeed = 0; // speed the lift already had when this press started

bool fading = false;
std::uint32_t fadeStart = 0;
double fadeFrom = 0;
double fadeTime = 0;

// Speed the driver is asking for after holding this many milliseconds
double heldSpeed(double heldMs, double topSpeed) {
  const double from = std::min(carriedSpeed, topSpeed);
  // Pressing again mid-stop continues from the current speed
  if (from >= TAP_SPEED) {
    return from + (topSpeed - from) * smoothStep(heldMs / ACCEL_TIME);
  }
  const double tapTop = std::min(TAP_SPEED, topSpeed);
  if (heldMs < TAP_TIME) {
    return from + (tapTop - from) * smoothStep(heldMs / TAP_TIME);
  }
  return tapTop +
         (topSpeed - tapTop) * smoothStep((heldMs - TAP_TIME) / ACCEL_TIME);
}

// Moves the commanded speed toward the target without jumping
void sendSpeed(double target, double maxChangePerSecond) {
  const double current = LiftCore::commandedSpeed();
  const double maxChange = maxChangePerSecond * LOOP_MS / 1000.0;
  LiftCore::setSpeed(current + std::clamp(target - current, -maxChange, maxChange));
}

} // namespace

double stoppingSpeed(bool goingUp, double room) {
  const double decel = goingUp ? TOP_DECEL : BOTTOM_DECEL;
  const double ease = goingUp ? TOP_EASE : BOTTOM_EASE;
  const double usableRoom =
      std::max(room - std::fabs(LiftCore::commandedSpeed()) * LOOK_AHEAD, 0.0);

  // Steady braking when far away, then a gentle fade for the last few degrees.
  // The two curves meet with the same slope, so there is no jolt between them
  const double easeRoom = decel / (ease * ease);
  const double curve = usableRoom <= easeRoom
                           ? ease * usableRoom
                           : std::sqrt(2 * decel * (usableRoom - easeRoom / 2));
  return LANDING_SPEED + curve;
}

void run(int power, double height) {
  const std::uint32_t now = pros::millis();
  const int direction = power > 0 ? 1 : -1;
  const bool goingUp = direction > 0;

  if (direction != holdDirection) {
    holdDirection = direction;
    holdStart = now;
    carriedSpeed =
        std::max(START_SPEED, direction * LiftCore::commandedSpeed());
    LiftCore::resetStall();
  }
  fading = false;

  // Only checks for the floor when pushing down near the bottom
  const bool justStarted = now - holdStart < STALL_GRACE;
  if (goingUp || height > FLOOR_ZONE || justStarted) {
    LiftCore::resetStall();
  } else if (LiftCore::isStalled(height, STALL_TIME)) {
    LiftCore::setPower(0);
    return;
  }

  const double room = LiftCore::roomToLimit(goingUp, height);
  if (room <= 0) {
    LiftCore::setPower(0);
    return;
  }

  const double topSpeed = std::abs(power) / 127.0 * FULL_SPEED;
  const double wanted = std::min(heldSpeed(now - holdStart, topSpeed),
                                 stoppingSpeed(goingUp, room));
  sendSpeed(goingUp ? wanted : -wanted, MAX_SPEED_CHANGE);
}

void beginRelease() {
  holdDirection = 0;
  fading = true;
  fadeStart = pros::millis();
  fadeFrom = LiftCore::commandedSpeed();
  // An S-curve's steepest point is 1.5 times its average slope
  fadeTime =
      std::max(RELEASE_MIN_TIME, 1500.0 * std::fabs(fadeFrom) / RELEASE_DECEL);
}

bool releasing() { return fading; }

bool runRelease(double height) {
  const double elapsed = pros::millis() - fadeStart;
  const bool goingUp = fadeFrom > 0;
  const double room = LiftCore::roomToLimit(goingUp, height);

  if (elapsed < fadeTime + RELEASE_SETTLE && room > 0) {
    const double limit = stoppingSpeed(goingUp, room);
    const double speed = fadeFrom * (1 - smoothStep(elapsed / fadeTime));
    sendSpeed(std::clamp(speed, -limit, limit), RELEASE_DECEL);
    return true;
  }

  // Stopped, so HOLD can take over without a bounce
  fading = false;
  LiftCore::setPower(0);
  return false;
}

void reset() {
  holdDirection = 0;
  fading = false;
}

} // namespace LiftDriver
