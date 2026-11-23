// MOVEMENT MACROS
// Cardinal turn: left and right arrows snap to the next 0, 90, 180, or 270.
// Tapping again during a turn moves the target another 90 without stopping,
// so left, left, right ends up only 90 degrees left of the start.
#include "../components/components.hpp"
#include "../hardware/hardware.hpp"
#include "../pid/turnpid.hpp"
#include "macro.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

namespace Macro {
namespace {

constexpr double CARDINAL = 90;
// Within this many degrees of a cardinal counts as already facing it,
// so a press always makes a real turn instead of a tiny correction
constexpr double ON_CARDINAL = 15;

// SPEED
constexpr double MAX_POWER = 127;
// Weakest power that still turns the robot, used until it is within tolerance
constexpr double MIN_POWER = 12;
// Most the power can change every loop, so the turn doesn't jerk
constexpr double ACCEL_STEP = 30;
constexpr double BRAKE_STEP = 70;

// FINISHING
constexpr double SETTLE_ERROR = 1;              // degrees from the target
constexpr double SETTLE_SPEED = 20;             // degrees per second
constexpr std::uint32_t SETTLE_TIME = 50;       // milliseconds inside both
constexpr std::uint32_t BASE_TIMEOUT = 700;     // milliseconds
constexpr std::uint32_t TIMEOUT_PER_TURN = 450; // added per 90 degrees left

// The PID has no timer of its own, so it needs the same loop LemLib uses
constexpr std::uint32_t LOOP_MS = 10;
// Longest cancelTurn waits for the task to let go of the motors
constexpr std::uint32_t CANCEL_WAIT = 40;

// Shared between the driver loop and the turn task
std::atomic<int> pendingTurns{0}; // + is right, - is left
std::atomic<bool> stopRequested{false};
std::atomic<bool> turning{false};

// Only the driver loop uses this
bool inputCleared = false;

// Only the turn task uses these
bool controlling = false;
double heading = 0; // keeps counting past 360 so long turns go the right way
double lastRaw = 0;
double target = 0;
double power = 0;
bool settling = false;
std::uint32_t settleStart = 0;
std::uint32_t deadline = 0;

double wrap180(double degrees) {
  return degrees - 360.0 * std::round(degrees / 360.0);
}

double firstTarget(double current, int steps) {
  const double nearest = std::round(current / CARDINAL) * CARDINAL;
  double base = nearest;
  if (std::fabs(current - nearest) > ON_CARDINAL) {
    base = (steps > 0 ? std::floor(current / CARDINAL)
                      : std::ceil(current / CARDINAL)) *
           CARDINAL;
  }
  return base + steps * CARDINAL;
}

void aimAtTarget(std::uint32_t now) {
  const double error = target - heading;
  TurnPid::restart(error); // no derivative kick from the target jumping
  settling = false;
  deadline = now + BASE_TIMEOUT +
             static_cast<std::uint32_t>(TIMEOUT_PER_TURN * std::fabs(error) /
                                        CARDINAL);
}

void finishTurn() {
  controlling = false;
  power = 0;
  chassis.tank(0, 0, true);
  turning = false;
}

// Moves toward the wanted power, with a faster limit for slowing down
void sendPower(double wanted) {
  const bool speedingUp =
      wanted * power >= 0 && std::fabs(wanted) > std::fabs(power);
  const double maxStep = speedingUp ? ACCEL_STEP : BRAKE_STEP;
  power += std::clamp(wanted - power, -maxStep, maxStep);
  const int output = static_cast<int>(std::lround(power));
  chassis.tank(output, -output, true); // positive turns right
}

void stepTurn() {
  if (stopRequested.load()) {
    pendingTurns = 0;
    if (turning.load()) {
      finishTurn();
    }
    stopRequested = false;
    return;
  }
  // Never keep driving into a disabled period or the match auton
  if (turning.load() && (pros::competition::is_disabled() ||
                         pros::competition::is_autonomous())) {
    pendingTurns = 0;
    finishTurn();
    return;
  }

  if (pendingTurns.load() != 0) {
    turning = true;
  }
  const int steps = pendingTurns.exchange(0);
  if (!turning.load()) {
    return;
  }

  const std::uint32_t now = pros::millis();
  const double raw = chassis.getPose().theta;

  if (!controlling) {
    // Without the IMU the robot can't tell when to stop turning
    if (steps == 0 || std::isinf(Hardware::imu.get_heading())) {
      turning = false;
      return;
    }
    controlling = true;
    heading = raw;
    lastRaw = raw;
    power = 0;
    target = firstTarget(heading, steps);
    aimAtTarget(now);
  } else if (steps != 0) {
    target += steps * CARDINAL;
    aimAtTarget(now);
  }

  const double change = wrap180(raw - lastRaw);
  lastRaw = raw;
  heading += change;
  const double speed = change * 1000.0 / LOOP_MS;
  const double error = target - heading;

  if (std::fabs(error) <= SETTLE_ERROR && std::fabs(speed) <= SETTLE_SPEED) {
    if (!settling) {
      settling = true;
      settleStart = now;
    }
    if (now - settleStart >= SETTLE_TIME) {
      finishTurn();
      return;
    }
  } else {
    settling = false;
  }
  if (static_cast<std::int32_t>(now - deadline) >= 0) {
    finishTurn();
    return;
  }

  double wanted = std::clamp(TurnPid::update(error), -MAX_POWER, MAX_POWER);
  if (std::fabs(error) > SETTLE_ERROR && wanted * error >= 0 &&
      std::fabs(wanted) < MIN_POWER) {
    wanted = std::copysign(MIN_POWER, error);
  }
  sendPower(wanted);
}

} // namespace

void startMovement() {
  static pros::Task turnTask(
      [] {
        while (true) {
          stepTurn();
          pros::delay(LOOP_MS);
        }
      },
      "Turn Macro");
}

bool turn(bool left, bool right, bool stickInput) {
  const int presses = (right ? 1 : 0) - (left ? 1 : 0);
  if (!turning.load() && pendingTurns.load() == 0) {
    if (presses == 0) {
      return false;
    }
    // A stick already held when the turn starts shouldn't cancel it
    inputCleared = !stickInput;
  } else if (!stickInput) {
    inputCleared = true;
  } else if (inputCleared) {
    cancelTurn();
    return false;
  }

  if (presses != 0) {
    pendingTurns.fetch_add(presses);
  }
  return true;
}

void cancelTurn() {
  if (!turning.load() && pendingTurns.load() == 0) {
    return;
  }
  stopRequested = true;
  const std::uint32_t start = pros::millis();
  while (stopRequested.load() && pros::millis() - start < CANCEL_WAIT) {
    pros::delay(2);
  }
  stopRequested = false;
}

} // namespace Macro
