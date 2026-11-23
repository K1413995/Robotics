// TOGGLE MACRO
#include "macro.hpp"
#include "../components/components.hpp"
#include "../hardware/hardware.hpp"
#include "pros/rtos.hpp"

#include <cstdint>

namespace Macro {
namespace {

// Lift angle to start from, in degrees, using the angle on the brain screen
constexpr double TOGGLE_LIFT_ANGLE = 21;
// How many degrees the lift rises after backing up
constexpr double TOGGLE_LIFT_RAISE = 20;
// Raw lift power for that raise, out of 127, with no ramp or slowing
constexpr int TOGGLE_RAISE_POWER = 127;
// Longest the raw raise can run, in milliseconds
constexpr std::uint32_t TOGGLE_RAISE_TIMEOUT = 500;
// Drive power for the backward and forward steps, out of 127
constexpr int TOGGLE_DRIVE_POWER = 30;
// Rough drive times in milliseconds, these are timed and not measured
// Raise them to travel farther, lower them to travel less
constexpr std::uint32_t TOGGLE_BACK_TIME = 240;    // about 2 inches
constexpr std::uint32_t TOGGLE_FORWARD_TIME = 110; // about half an inch
// Longest wait for the lift before moving to the next step
constexpr std::uint32_t TOGGLE_LIFT_TIMEOUT = 1200;

enum class Step { Idle, LiftDown, DriveBack, LiftUp, LiftSettle, DriveForward };

Step step = Step::Idle;
std::uint32_t stepStart = 0;
bool inputCleared = false;

void enterStep(Step next) {
  step = next;
  stepStart = pros::millis();
}

void driveStraight(int power) { chassis.tank(power, power, true); }

bool liftFinished() {
  return Components::liftSettled() ||
         pros::millis() - stepStart >= TOGGLE_LIFT_TIMEOUT;
}

void runStep() {
  const std::uint32_t elapsed = pros::millis() - stepStart;

  switch (step) {
  case Step::LiftDown:
    driveStraight(0);
    if (liftFinished()) {
      enterStep(Step::DriveBack);
    }
    break;

  case Step::DriveBack:
    if (elapsed < TOGGLE_BACK_TIME) {
      driveStraight(-TOGGLE_DRIVE_POWER);
    } else {
      driveStraight(0);
      enterStep(Step::LiftUp);
    }
    break;

  case Step::LiftUp: {
    // Raw speed so the lift can flip the toggle
    driveStraight(0);
    const std::int32_t angle = Hardware::liftRotation.get_angle();
    const bool reached = angle != PROS_ERR &&
                         angle / 100.0 >= TOGGLE_LIFT_ANGLE + TOGGLE_LIFT_RAISE;
    if (angle == PROS_ERR || reached || elapsed >= TOGGLE_RAISE_TIMEOUT) {
      Components::liftRaw(0);
      enterStep(Step::LiftSettle);
    } else {
      Components::liftRaw(TOGGLE_RAISE_POWER);
    }
    break;
  }

  case Step::LiftSettle:
    driveStraight(0);
    if (liftFinished()) {
      enterStep(Step::DriveForward);
    }
    break;

  case Step::DriveForward:
    if (elapsed < TOGGLE_FORWARD_TIME) {
      driveStraight(TOGGLE_DRIVE_POWER);
    } else {
      driveStraight(0);
      step = Step::Idle;
    }
    break;

  case Step::Idle:
    break;
  }
}

} // namespace

bool toggle(bool pressed, bool driverInput) {
  if (step == Step::Idle) {
    if (!pressed) {
      return false;
    }
    Components::lift(0);
    Components::liftTo(TOGGLE_LIFT_ANGLE);
    // A stick already held against the wall shouldn't cancel the macro
    inputCleared = !driverInput;
    enterStep(Step::LiftDown);
  } else if (!driverInput) {
    inputCleared = true;
  } else if (inputCleared) {
    cancelToggle();
    return false;
  }

  runStep();
  return step != Step::Idle;
}

void cancelToggle() {
  if (step == Step::Idle) {
    return;
  }
  step = Step::Idle;
  driveStraight(0);
  Components::liftRaw(0);

  const std::int32_t angle = Hardware::liftRotation.get_angle();
  if (angle != PROS_ERR) {
    Components::liftTo(angle / 100.0);
  }
}

} // namespace Macro
