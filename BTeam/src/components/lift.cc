// LIFT
// Picks who controls the lift each loop:
//   Driver holding a button -> LiftDriver smoothing (controller/liftdriver.cc)
//   Macro or stick power    -> straight to the motors, but going down it
//                              slows for the bottom like the buttons
//   Button just released    -> LiftDriver fades the speed out
//   Everything else         -> LiftPid goes to or holds a target (pid/liftpid.cc)
// Driver tuning is in liftconfig.hpp, PID tuning is in pid/pidconfig.hpp
// Lift brake mode is HOLD, set in main.cc initialize()
#include "../controller/liftdriver.hpp"
#include "../hardware/hardware.hpp"
#include "../pid/liftpid.hpp"
#include "components.hpp"
#include "liftconfig.hpp"
#include "liftcore.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>

static constexpr int NO_REQUEST = INT32_MIN;

// Shared between the driver or auton code and the lift task
static std::atomic<int> driverPower{0};
static std::atomic<int> requestedHeight{NO_REQUEST}; // tenths of a degree
static std::atomic<int> rawPower{0};                 // set by macros
static std::atomic<bool> liftDone{true};

// Only the lift task uses these
static bool handMoving = false; // the driver or a macro moved it last loop
static bool rawSlowing = false; // raw power going down hit the bottom slowing

// Lift speed in degrees per second, positive is up
static double measuredSpeed() {
  return Hardware::liftMotors.get_actual_velocity() * LiftConfig::GEAR_RATIO *
         6;
}

// Raw power (the lift stick) going down slows for the bottom like the buttons.
// Far from the bottom it stays raw. Once the slowing starts it keeps the speed
// in control until the stick is let go, so gravity can't speed it back up
static void runRaw(int raw, double height) {
  const bool goingUp = raw > 0;
  const double room = LiftCore::roomToLimit(goingUp, height);
  if (room <= 0) {
    rawSlowing = false;
    LiftCore::setPower(0);
    return;
  }
  if (goingUp) {
    rawSlowing = false;
    LiftCore::setPower(raw);
    return;
  }
  const double asked = -raw / 127.0 * LiftConfig::FULL_SPEED;
  const double limit = LiftDriver::stoppingSpeed(false, room);
  if (asked > limit || -measuredSpeed() > limit) {
    rawSlowing = true;
  }
  if (rawSlowing) {
    LiftCore::setSpeed(-std::min(asked, limit));
  } else {
    LiftCore::setPower(raw);
  }
}

static void stepLift() {
  const std::int32_t angle = Hardware::liftRotation.get_angle();
  const int driver = driverPower.load();
  int request = NO_REQUEST;
  if (requestedHeight.load() != NO_REQUEST) {
    liftDone = false;
    request = requestedHeight.exchange(NO_REQUEST);
  }

  // Sensor unplugged: no PID or limits, but the driver can still use the lift
  if (angle == PROS_ERR) {
    const int power = driver != 0 ? driver : rawPower.load();
    LiftPid::clear();
    LiftDriver::reset();
    handMoving = power != 0;
    liftDone = true;
    LiftCore::setPower(power);
    return;
  }
  const double height = LiftCore::angleToHeight(angle / 100.0);

  if (driver != 0) {
    LiftPid::clear();
    handMoving = true;
    liftDone = true;
    LiftDriver::run(driver, height);
    return;
  }

  const int raw = rawPower.load();
  if (raw != 0) {
    LiftPid::clear();
    LiftDriver::reset();
    handMoving = true; // hold wherever it stops once raw power ends
    liftDone = false;
    runRaw(raw, height);
    return;
  }
  rawSlowing = false;

  if (handMoving) {
    handMoving = false;
    LiftPid::clear();
    LiftDriver::beginRelease();
  }
  if (request != NO_REQUEST) {
    LiftDriver::reset();
    LiftPid::setTarget(request / 10.0, height);
  }
  if (LiftDriver::releasing()) {
    if (LiftDriver::runRelease(height)) {
      return;
    }
    LiftPid::setTarget(height, height);
  }
  if (LiftPid::run(height)) {
    liftDone = true;
  }
}

static void requestHeight(double height) {
  const double limited = std::clamp(height, 0.0, LiftCore::range());
  liftDone = false;
  requestedHeight = static_cast<int>(std::lround(limited * 10));
}

void Components::startLift() {
  static pros::Task liftTask(
      [] {
        while (true) {
          stepLift();
          pros::delay(LiftConfig::LOOP_MS);
        }
      },
      "Lift");
}

// Driver control: positive is up, negative is down, 0 holds the current spot
void Components::lift(int voltage) { driverPower = voltage; }

// Moves to a brain-screen angle, staying inside the limits
void Components::liftTo(double angle) {
  requestHeight(LiftCore::angleToHeight(angle));
}

// Moves up (positive) or down (negative) from where the lift is now
void Components::liftBy(double degrees) {
  const std::int32_t angle = Hardware::liftRotation.get_angle();
  if (angle != PROS_ERR) {
    requestHeight(LiftCore::angleToHeight(angle / 100.0) + degrees);
  }
}

// Raw motor power for macros and the lift stick, 0 stops and holds in place
void Components::liftRaw(int power) {
  if (power != 0) {
    liftDone = false;
  }
  rawPower = power;
}

// True once the lift has arrived at its target or gotten blocked
bool Components::liftSettled() {
  return requestedHeight.load() == NO_REQUEST && rawPower.load() == 0 &&
         liftDone.load();
}

// Waits until the lift arrives, gets blocked, or runs out of time
void Components::waitForLift(int timeoutMs) {
  const std::uint32_t start = pros::millis();
  while (!liftSettled() &&
         pros::millis() - start < static_cast<std::uint32_t>(timeoutMs)) {
    pros::delay(LiftConfig::LOOP_MS);
  }
}
