#include "controller.hpp"
#include "../components/components.hpp"
#include "../hardware/hardware.hpp"
#include "../lemlib/drivetrain.hpp"
#include "../macro/macro.hpp"

#include <cstdlib>

namespace Controller {

// Moving a stick past this cancels a running macro
constexpr int MACRO_CANCEL_STICK = 15;

void execute(const Controls &driver) {
  int drive = Hardware::controller.get_analog(driver.leftY);
  int steer = Hardware::controller.get_analog(driver.rightX);

  // macros control the drive and lift until they finish or get canceled
  const bool driverInput = std::abs(drive) > MACRO_CANCEL_STICK ||
                           std::abs(steer) > MACRO_CANCEL_STICK ||
                           driver.liftUp.pressed() || driver.liftDown.pressed();
  const bool macroRunning =
      Macro::toggle(driver.toggleMacro.newPress(), driverInput);

  if (!macroRunning) {
    // drive
    chassis.arcade(drive, steer);

    // lift
    if (driver.liftUp.pressed()) {
      Components::lift(95);
    } else if (driver.liftDown.pressed()) {
      Components::lift(-95);
    } else {
      Components::lift(0);
    }
  }

  // claw
  Components::clawIntake(driver.clawIntake.pressed());
  Components::clawOpen(driver.clawOpen.released());
}
} // namespace Controller
