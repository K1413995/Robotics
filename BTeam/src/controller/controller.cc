#include "controller.hpp"
#include "../components/components.hpp"
#include "../hardware/hardware.hpp"
#include "../macro/macro.hpp"
#include "liftstick.hpp" // LIFT STICK: remove this line

#include <cstdlib>

namespace Controller {

// Moving a stick past this cancels a running macro
constexpr int MACRO_CANCEL_STICK = 15;

void execute(const Controls &driver) {
  const int drive = driver.leftY.value();
  // LIFT STICK: remove the next line and use driver.rightX.value() for steer
  const LiftStick::Output stick = LiftStick::read(driver);
  const int steer = stick.steer;

  // macros control the drive and lift until they finish or get canceled
  const bool stickInput =
      driver.leftY.past(MACRO_CANCEL_STICK) ||
      std::abs(steer) > MACRO_CANCEL_STICK;
  const bool liftInput = driver.liftUp.pressed() || driver.liftDown.pressed() ||
                         stick.liftPower != 0; // LIFT STICK: remove this
  const bool driverInput = stickInput || liftInput;

  // The toggle needs the drive, so it takes over from a turn
  if (driver.toggleMacro.newPress()) {
    Macro::cancelTurn();
  }
  const bool toggleRunning =
      Macro::toggle(driver.toggleMacro.newPress(), driverInput, liftInput);
  const bool turning =
      !toggleRunning && Macro::turn(driver.turnLeft.newPress(),
                                    driver.turnRight.newPress(), stickInput);

  if (!toggleRunning) {
    // drive
    if (!turning) {
      chassis.arcade(drive, steer);
    }

    // lift still works during a turn
    if (driver.liftUp.pressed()) {
      Components::lift(95);
    } else if (driver.liftDown.pressed()) {
      Components::lift(-95);
    } else {
      Components::lift(0);
    }
    Components::liftRaw(stick.liftPower); // LIFT STICK: remove this line
  }

  // claw: on one shared button, a tap opens and a hold runs the intake
  const bool shared = driver.clawIntake.id() == driver.clawOpen.id();
  Components::clawIntake(shared ? driver.clawIntake.held()
                                : driver.clawIntake.pressed());
  if (shared ? driver.clawOpen.tapped() : driver.clawOpen.released()) {
    Components::clawOpen();
  }
}
} // namespace Controller
