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
// After the last toggle finishes, the lift goes back to where it was before
// the first toggle once this much time passes with no new toggle, in ms
constexpr std::uint32_t TOGGLE_RETURN_DELAY = 3000;

enum class Step { Idle, LiftDown, DriveBack, LiftUp, LiftSettle, DriveForward };
// None: nothing to do. Waiting: counting down. Moving: the lift is going back
enum class Return { None, Waiting, Moving };

Step step = Step::Idle;
std::uint32_t stepStart = 0;
bool inputCleared = false;

Return returnState = Return::None;
double returnAngle = 0;     // lift angle before the first toggle in a row
bool haveReturnAngle = false; // false if the sensor couldn't read it
std::uint32_t finishedAt = 0;

// A toggle in quick succession keeps the angle from before the first one
void rememberReturnAngle() {
  const bool chained =
      returnState == Return::Waiting ||
      (returnState == Return::Moving && !Components::liftSettled());
  returnState = Return::None;
  if (chained) {
    return;
  }
  const std::int32_t angle = Hardware::liftRotation.get_angle();
  haveReturnAngle = angle != PROS_ERR;
  returnAngle = angle / 100.0;
}

// Runs while no toggle is running. Using the lift drops the return
void updateReturn(bool liftInput) {
  if (returnState == Return::None) {
    return;
  }
  if (liftInput) {
    returnState = Return::None;
  } else if (returnState == Return::Waiting &&
             pros::millis() - finishedAt >= TOGGLE_RETURN_DELAY) {
    Components::liftTo(returnAngle);
    returnState = Return::Moving;
  } else if (returnState == Return::Moving && Components::liftSettled()) {
    returnState = Return::None;
  }
}

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
      if (haveReturnAngle) {
        returnState = Return::Waiting;
        finishedAt = pros::millis();
      }
    }
    break;

  case Step::Idle:
    break;
  }
}

} // namespace

bool toggle(bool pressed, bool driverInput, bool liftInput) {
  if (step == Step::Idle) {
    if (!pressed) {
      updateReturn(liftInput);
      return false;
    }
    rememberReturnAngle();
    Components::lift(0);
    Components::liftRaw(0); // a held lift stick can't fight the macro
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
  returnState = Return::None; // a canceled toggle doesn't go back
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
