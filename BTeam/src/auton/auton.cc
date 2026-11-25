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

// How long to wait after opening the claw before driving away, in ms.
// The claw keeps opening for its full 2 seconds while the robot drives.
constexpr std::uint32_t CLAW_RELEASE_WAIT = 500;

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
  // Forward and right to (120, -55) cm, ending 90 degrees right (heading 0)
  moveTo(120, -55, 0, 3000);
  if (!finished()) {
    return;
  }

  // Let go, and give the claw a moment to open before driving away
  Components::clawOpen();
  if (!AutonRun::wait(CLAW_RELEASE_WAIT)) {
    return;
  }

  // Back near the start at (150, -150) cm, turned around to face our wall
  // (heading 90) for the match load
  moveTo(150, -150, 90, 3000);
  if (!finished()) {
    return;
  }
}
