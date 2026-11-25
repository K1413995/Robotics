#include "auton.hpp"
#include "../components/components.hpp"
#include "../hardware/hardware.hpp"
#include "../interface/settings.hpp"
#include "autonrun.hpp"
#include "position.hpp"

#include <cmath>
#include <cstdint>

// Jerry.io shows centimeters by default, LemLib uses inches
constexpr float cm(float centimeters) { return centimeters / 2.54f; }

// Lift angles for the first score, in brain-screen degrees
constexpr double CARRY_LIFT_ANGLE = 25.5; // raised while driving to the goal
constexpr double SCORE_LIFT_ANGLE = 18;   // lowered onto the goal
// Longest wait for the lift to arrive, in milliseconds
constexpr std::uint32_t LIFT_WAIT = 1000;

static bool onRightSide() { return Settings::side() == Settings::Side::Right; }

// Write auton points for the LEFT side, the right side mirrors them.
// Mirroring flips left/right (Y) only. X, toward the enemy, stays the same.
static float sideY(float y) { return onRightSide() ? -y : y; }
// Turns a left side heading into the mirrored one: 0 and 180 swap,
// 90 (our wall) and 270 (the enemy) stay the same
static float sideHeading(float heading) {
  return onRightSide() ? std::fmod(540.0f - heading, 360.0f) : heading;
}

// Moves to a LEFT side point in centimeters, flipped for the right side
static void moveTo(float xCm, float yCm, float heading, int timeout,
                   lemlib::MoveToPoseParams params = {}) {
  chassis.moveToPose(cm(xCm), sideY(cm(yCm)), sideHeading(heading), timeout,
                     params);
}

// Drives to a LEFT side point in centimeters without a set ending heading
static void pointTo(float xCm, float yCm, int timeout,
                    lemlib::MoveToPointParams params = {}) {
  chassis.moveToPoint(cm(xCm), sideY(cm(yCm)), timeout, params);
}

// Turns in place to a LEFT side heading, flipped for the right side
static void turnTo(float heading, int timeout) {
  chassis.turnToHeading(sideHeading(heading), timeout);
}

// Waits for the lift like finished() waits for the drive
static bool liftFinished(std::uint32_t timeoutMs = LIFT_WAIT) {
  const std::uint32_t start = pros::millis();
  while (!Components::liftSettled() && pros::millis() - start < timeoutMs) {
    if (!AutonRun::wait(10)) {
      return false;
    }
  }
  return !AutonRun::canceled();
}

// Reads the walls using the side picked in the controller menu
bool setStartPose() {
  return Position::setPos(AutonConfig::START_HEADING,
                          onRightSide() ? "right" : "left");
}

// Waits for the current move. False means the test code canceled the auton
static bool finished() {
  chassis.waitUntilDone();
  return !AutonRun::canceled();
}

// All points are LEFT side field coordinates in cm, see the plane in auton.hpp
// Use moveTo for every move so the right side flips automatically
// After every move use: if (!finished()) return;
// For pauses use: if (!AutonRun::wait(500)) return;
void auton() {
  // Forward and right to (120, -55) cm, ending 90 degrees right (heading 0),
  // raising the lift on the way
  moveTo(120, -55, 0, 3000);
  Components::liftTo(CARRY_LIFT_ANGLE);
  if (!finished()) {
    return;
  }

  // Lower onto the goal
  Components::liftTo(SCORE_LIFT_ANGLE);
  if (!liftFinished()) {
    return;
  }

  // Let go and drive off right away, the claw keeps opening on its own.
  // Backs straight to (150, -150) cm near the start...
  Components::clawOpen();
  pointTo(150, -150, 3000, {.forwards = false});
  if (!finished()) {
    return;
  }
  // ...then turns to face our wall (heading 90) for the match load
  turnTo(90, 1500);
  if (!finished()) {
    return;
  }
}
