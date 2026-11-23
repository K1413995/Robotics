#include "controls.hpp"
#include "../hardware/hardware.hpp"

void Button::update() {
  bool current = Hardware::controller.get_digital(control);

  std::uint32_t currentTime = pros::millis();

  pressedEvent = current && !active;
  releasedEvent = !current && active;
  tapEvent = false;

  if (pressedEvent) {
    startedHolding = currentTime;
  }

  if (releasedEvent) {
    tapEvent = (currentTime - startedHolding) < holdThreshold;
  }

  active = current;
}

bool Button::held() const {
  return active && (pros::millis() - startedHolding >= holdThreshold);
}

void Controls::update(Controls &driver) {
  driver.liftUp.update();
  driver.liftDown.update();
  driver.toggleMacro.update();
  driver.turnLeft.update();
  driver.turnRight.update();

  driver.clawIntake.update();
  driver.clawOpen.update();

  driver.menuUp.update();
  driver.menuDown.update();
  driver.menuLeft.update();
  driver.menuRight.update();
  driver.menuSelect.update();
  driver.menuBack.update();

  driver.autoMode.update();
  driver.driverMode.update();
  driver.pidMode.update();
}
