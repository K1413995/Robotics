// SETTINGS
// Pre-match menu on the controller screen, only in the test code
// Up and down pick a setting, left and right change it, A confirms
#include "settings.hpp"
#include "../auton/auton.hpp"
#include "../hardware/hardware.hpp"
#include "ctrlscreen.hpp"
#include "pros/misc.hpp"

#include <cstdio>
#include <cstdlib>

namespace Settings {
namespace {

// Moving a stick past this closes the menu so the driver is never locked out
constexpr int STICK_DEADBAND = 15;

struct Setting {
  const char *name;
  const char *options[4];
  int optionCount;
  int selected;
};

// ADD NEW SETTINGS HERE, the first option is the default
enum SettingIndex { SIDE_SETTING };
Setting settings[] = {
    {"Side", {"LEFT", "RIGHT"}, 2, 0},
};
constexpr int SETTING_COUNT = sizeof(settings) / sizeof(settings[0]);

bool menuOpen = false;
bool menuUsed = false;
bool positionFound = false;
int row = 0;

void drawMenu() {
  const Setting &setting = settings[row];
  char text[32];
  std::snprintf(text, sizeof(text), "%s: <%s>", setting.name,
                setting.options[setting.selected]);
  CtrlScreen::write(0, text);
  CtrlScreen::write(1, "Arrows to edit");
  CtrlScreen::write(2, "A to confirm");
}

void moveRow(int change) {
  row = (row + change + SETTING_COUNT) % SETTING_COUNT;
  drawMenu();
}

void changeOption(int change) {
  Setting &setting = settings[row];
  setting.selected =
      (setting.selected + change + setting.optionCount) % setting.optionCount;
  drawMenu();
}

// Reads the walls for the chosen side, then reports the result
void closeMenu() {
  menuOpen = false;
  CtrlScreen::rumble(readPosition() ? "." : "---");
}

bool isMenuButton(const Button &button, const Controls &driver) {
  const pros::controller_digital_e_t id = button.id();
  return id == driver.menuUp.id() || id == driver.menuDown.id() ||
         id == driver.menuLeft.id() || id == driver.menuRight.id() ||
         id == driver.menuSelect.id();
}

bool robotButtonPressed(const Button &button, const Controls &driver) {
  return button.pressed() && !isMenuButton(button, driver);
}

// If the driver forgets to confirm, using the robot closes the menu
bool driverTookOver(const Controls &driver) {
  return driver.leftY.past(STICK_DEADBAND) ||
         driver.rightX.past(STICK_DEADBAND) ||
         driver.rightY.past(STICK_DEADBAND) ||
         robotButtonPressed(driver.liftUp, driver) ||
         robotButtonPressed(driver.liftDown, driver) ||
         robotButtonPressed(driver.toggleMacro, driver) ||
         robotButtonPressed(driver.turnLeft, driver) ||
         robotButtonPressed(driver.turnRight, driver) ||
         robotButtonPressed(driver.clawIntake, driver) ||
         robotButtonPressed(driver.clawOpen, driver) ||
         driver.driverMode.pressed() || driver.pidMode.pressed();
}

} // namespace

Side side() {
  return settings[SIDE_SETTING].selected == 0 ? Side::Left : Side::Right;
}

void setSide(Side next) {
  settings[SIDE_SETTING].selected = next == Side::Left ? 0 : 1;
}

bool readPosition() {
  positionFound = setStartPose();
  return positionFound;
}

bool positionOk() { return positionFound; }

void open() {
  // Only once per program, and never with a competition switch or field,
  // because a second wall reading mid-match would overwrite the position
  if (menuUsed || pros::competition::is_connected()) {
    return;
  }
  menuUsed = true;
  menuOpen = true;
  row = 0;
  drawMenu();
}

bool update(const Controls &driver) {
  if (!menuOpen) {
    return false;
  }

  if (driverTookOver(driver)) {
    closeMenu();
    return false;
  }
  if (driver.menuSelect.newPress()) {
    closeMenu();
    return true;
  }

  if (driver.menuUp.newPress()) {
    moveRow(-1);
  }
  if (driver.menuDown.newPress()) {
    moveRow(1);
  }
  if (driver.menuLeft.newPress()) {
    changeOption(-1);
  }
  if (driver.menuRight.newPress()) {
    changeOption(1);
  }
  return true;
}

} // namespace Settings
