#include "auton.hpp"
#include "../components/components.hpp"
#include "../hardware/hardware.hpp"
#include "../interface/settings.hpp"
#include "../lemlib/drivetrain.hpp"
#include "position.hpp"

#include <cmath>

// Jerry.io shows centimeters by default, LemLib uses inches
constexpr float cm(float centimeters) { return centimeters / 2.54f; }

static bool onRightSide() { return Settings::side() == Settings::Side::Right; }

// Write auton points for the LEFT side, the right side mirrors them
static float sideY(float y) { return onRightSide() ? -y : y; }
static float sideHeading(float heading) {
  return onRightSide() ? std::fmod(540.0f - heading, 360.0f) : heading;
}

// Reads the walls using the side picked in the controller menu
bool setStartPose() {
  return Position::setPos(AutonConfig::START_HEADING,
                          onRightSide() ? "right" : "left");
}

// All points are field coordinates, see the plane in auton.hpp
void auton() {
  // Left side: forward and right to (120, -75) cm, ending 90 degrees right
  chassis.moveToPose(cm(120), sideY(cm(-55)), sideHeading(0), 3000);
  chassis.waitUntilDone();
}
