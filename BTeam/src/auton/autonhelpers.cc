#include "autonhelpers.hpp"
#include "../hardware/hardware.hpp"
#include "autonrun.hpp"
#include "pros/rtos.hpp"

#include <cmath>

namespace Auton {
namespace {
Settings::Side mirrorSide = Settings::Side::Left;

bool onRightSide() { return mirrorSide == Settings::Side::Right; }
} // namespace

void begin(Settings::Side side) { mirrorSide = side; }

float sideY(float y) { return onRightSide() ? -y : y; }

float sideHeading(float heading) {
  return onRightSide() ? std::fmod(540.0f - heading, 360.0f) : heading;
}

void moveTo(float xCm, float yCm, float heading, int timeout,
            lemlib::MoveToPoseParams params) {
  chassis.moveToPose(cm(xCm), sideY(cm(yCm)), sideHeading(heading), timeout,
                     params);
}

void pointTo(float xCm, float yCm, int timeout,
             lemlib::MoveToPointParams params) {
  chassis.moveToPoint(cm(xCm), sideY(cm(yCm)), timeout, params);
}

void turnTo(float heading, int timeout, lemlib::TurnToHeadingParams params) {
  chassis.turnToHeading(sideHeading(heading), timeout, params);
}

// Relative to the robot itself, so nothing needs flipping
void driveStraight(float distanceCm, int timeout,
                   lemlib::MoveToPointParams params) {
  const lemlib::Pose pose = chassis.getPose();
  const float radians = pose.theta * static_cast<float>(M_PI) / 180.0f;
  const float distance = cm(distanceCm);
  params.forwards = distance >= 0;
  chassis.moveToPoint(pose.x + distance * std::sin(radians),
                      pose.y + distance * std::cos(radians), timeout, params);
}

bool finished() {
  chassis.waitUntilDone();
  return !AutonRun::canceled();
}

bool liftFinished(std::uint32_t timeoutMs) {
  const std::uint32_t start = pros::millis();
  while (!Components::liftSettled() && pros::millis() - start < timeoutMs) {
    if (!AutonRun::wait(10)) {
      return false;
    }
  }
  return !AutonRun::canceled();
}

bool liftNear(double angle, double toleranceDeg, std::uint32_t timeoutMs) {
  const std::uint32_t start = pros::millis();
  while (pros::millis() - start < timeoutMs) {
    const std::int32_t raw = Hardware::liftRotation.get_angle();
    // Without the sensor, fall back to the lift's own settled check
    const bool close = raw == PROS_ERR
                           ? Components::liftSettled()
                           : std::fabs(raw / 100.0 - angle) <= toleranceDeg;
    if (close) {
      break;
    }
    if (!AutonRun::wait(10)) {
      return false;
    }
  }
  return !AutonRun::canceled();
}

bool waitUntilNear(float xCm, float yCm, float rangeCm) {
  const float x = cm(xCm);
  const float y = sideY(cm(yCm));
  while (chassis.isInMotion()) {
    const lemlib::Pose pose = chassis.getPose();
    if (std::hypot(pose.x - x, pose.y - y) <= cm(rangeCm)) {
      break;
    }
    if (!AutonRun::wait(10)) {
      return false;
    }
  }
  return !AutonRun::canceled();
}

bool wait(std::uint32_t ms) { return AutonRun::wait(ms); }

void after(std::uint32_t ms, std::function<void()> action) {
  pros::Task task(
      [ms, action] {
        if (AutonRun::wait(ms)) {
          action();
        }
      },
      "Auton After");
}

bool pushForward(int power, std::uint32_t ms) {
  chassis.tank(power, power, true);
  const bool ok = AutonRun::wait(ms);
  chassis.tank(0, 0, true);
  return ok;
}

} // namespace Auton
