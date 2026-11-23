// LIFT CORE
// Sensor math, motor output, and stall checking
#include "liftcore.hpp"
#include "../hardware/hardware.hpp"
#include "liftconfig.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>

namespace LiftCore {
namespace {

double speedSent = 0;
bool motorsBraked = false;
bool watchingStall = false;
double stallHeight = 0;
std::uint32_t stallStart = 0;

double wrapDegrees(double degrees) {
  return std::fmod(std::fmod(degrees, 360.0) + 360.0, 360.0);
}

} // namespace

double range() {
  return wrapDegrees(LiftConfig::ANGLE_DIRECTION *
                     (LiftConfig::MAX_ANGLE - LiftConfig::MIN_ANGLE));
}

double angleToHeight(double angle) {
  const double liftRange = range();
  double height =
      wrapDegrees(LiftConfig::ANGLE_DIRECTION * (angle - LiftConfig::MIN_ANGLE));
  // A little below the bottom should not look like almost a full circle up
  if (height > liftRange + (360.0 - liftRange) / 2) {
    height -= 360.0;
  }
  return height;
}

double roomToLimit(bool goingUp, double height) {
  return goingUp ? range() - height : height;
}

double smoothStep(double t) {
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3 - 2 * t);
}

void setPower(int power) {
  speedSent = 0;
  if (power != 0) {
    Hardware::liftMotors.move(power);
    motorsBraked = false;
  } else if (!motorsBraked) {
    Hardware::liftMotors.brake();
    motorsBraked = true;
  }
}

void setSpeed(double liftSpeed) {
  speedSent = liftSpeed;
  const double motorRpm = liftSpeed / (LiftConfig::GEAR_RATIO * 6);
  Hardware::liftMotors.move_velocity(static_cast<int>(std::lround(motorRpm)));
  motorsBraked = false;
}

double commandedSpeed() { return speedSent; }

bool isStalled(double height, std::uint32_t stallLimit) {
  const std::uint32_t now = pros::millis();
  if (!watchingStall ||
      std::fabs(height - stallHeight) > LiftConfig::STALL_MOVEMENT) {
    watchingStall = true;
    stallHeight = height;
    stallStart = now;
    return false;
  }
  return now - stallStart >= stallLimit;
}

void resetStall() { watchingStall = false; }

} // namespace LiftCore
