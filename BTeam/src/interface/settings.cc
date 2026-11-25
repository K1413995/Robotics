// SETTINGS
// Pre-match menu on the controller screen, only in the test code
// Up and down pick a setting (auton or side), left and right change it,
// A confirms
#include "settings.hpp"
#include "../auton/auton.hpp"
#include "../hardware/hardware.hpp"
#include "ctrlscreen.hpp"
#include "pros/misc.hpp"

#include <algorithm>
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

// ADD NEW SETTINGS HERE, the first option is the default.
// Option order matches the enums in settings.hpp
enum SettingIndex { ROUTINE_SETTING, SIDE_SETTING };
Setting settings[] = {
    {"Auto", {"MATCH", "SKILLS"}, 2, 0},
    {"Side", {"LEFT", "RIGHT"}, 2, 0},
};
constexpr int SETTING_COUNT = sizeof(settings) / sizeof(settings[0]);
// The controller has 3 lines, the last one is the help line
constexpr int VISIBLE_ROWS = 2;

bool menuOpen = false;
bool menuUsed = false;
bool positionFound = false;
int row = 0;

// Shows two settings at a time, with > on the one the arrows change
void drawMenu() {
  const int first = std::min(row, std::max(SETTING_COUNT - VISIBLE_ROWS, 0));
  char text[32];
  for (int line = 0; line < VISIBLE_ROWS; line++) {
    const int index = first + line;
    if (index >= SETTING_COUNT) {
      CtrlScreen::write(line, "");
      continue;
    }
    const Setting &setting = settings[index];
    std::snprintf(text, sizeof(text), "%c%s <%s>", index == row ? '>' : ' ',
                  setting.name, setting.options[setting.selected]);
    CtrlScreen::write(line, text);
  }
  CtrlScreen::write(2, "^v <> A=ok");
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

Routine routine() {
  return settings[ROUTINE_SETTING].selected == 0 ? Routine::Match
                                                 : Routine::Skills;
}

void setRoutine(Routine next) {
  settings[ROUTINE_SETTING].selected = next == Routine::Match ? 0 : 1;
}

const char *routineName(Routine which) {
  return settings[ROUTINE_SETTING].options[which == Routine::Match ? 0 : 1];
}

bool readPosition() {
  positionFound = Auton::setStartPose(side());
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
