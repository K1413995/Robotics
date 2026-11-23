// SETTINGS
// Pre-match menu on the controller screen
// Up and down pick a setting, left and right change it, L1 confirms
#include "settings.hpp"
#include "../auton/auton.hpp"
#include "../hardware/hardware.hpp"
#include "pros/misc.hpp"

#include <cstdint>
#include <cstdio>
#include <cstdlib>

namespace Settings {
namespace {

// The controller screen is slow, so space out updates
constexpr std::uint32_t SCREEN_DELAY_MS = 60;
// Moving a stick past this closes the menu so the driver is never locked out
constexpr int STICK_DEADBAND = 15;
constexpr int SCREEN_LINES = 3;
constexpr int SCREEN_WIDTH = 15;

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
int row = 0;
char screen[SCREEN_LINES][SCREEN_WIDTH + 1] = {};
bool screenChanged[SCREEN_LINES] = {};
const char *pendingRumble = nullptr;
std::uint32_t lastScreenTime = 0;

void writeLine(int line, const char *text) {
  std::snprintf(screen[line], sizeof(screen[line]), "%-*s", SCREEN_WIDTH, text);
  screenChanged[line] = true;
}

// Sends one update at a time to the controller
void updateScreen() {
  const std::uint32_t now = pros::millis();
  if (now - lastScreenTime < SCREEN_DELAY_MS) {
    return;
  }
  if (pendingRumble != nullptr) {
    Hardware::controller.rumble(pendingRumble);
    pendingRumble = nullptr;
    lastScreenTime = now;
    return;
  }
  for (int line = 0; line < SCREEN_LINES; line++) {
    if (screenChanged[line]) {
      if (Hardware::controller.set_text(line, 0, screen[line]) != PROS_ERR) {
        screenChanged[line] = false;
      }
      lastScreenTime = now;
      return;
    }
  }
}

void drawMenu() {
  const Setting &setting = settings[row];
  char text[32];
  std::snprintf(text, sizeof(text), "%s: <%s>", setting.name,
                setting.options[setting.selected]);
  writeLine(0, text);
  writeLine(1, "Arrows to edit");
  writeLine(2, "L1 to confirm");
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
  const bool found = setStartPose();

  const Setting &sideSetting = settings[SIDE_SETTING];
  char text[32];
  std::snprintf(text, sizeof(text), "Side: %s",
                sideSetting.options[sideSetting.selected]);
  writeLine(0, text);
  writeLine(1, found ? "Position OK" : "CHECK POSITION");
  writeLine(2, "");
  pendingRumble = found ? "." : "---";
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
  return std::abs(Hardware::controller.get_analog(driver.leftY)) >
             STICK_DEADBAND ||
         std::abs(Hardware::controller.get_analog(driver.rightX)) >
             STICK_DEADBAND ||
         robotButtonPressed(driver.liftUp, driver) ||
         robotButtonPressed(driver.liftDown, driver) ||
         robotButtonPressed(driver.toggleMacro, driver) ||
         robotButtonPressed(driver.clawIntake, driver) ||
         robotButtonPressed(driver.clawOpen, driver);
}

} // namespace

Side side() {
  return settings[SIDE_SETTING].selected == 0 ? Side::Left : Side::Right;
}

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
  updateScreen();
  if (!menuOpen) {
    return false;
  }

  if (driver.auton.newPress() || driverTookOver(driver)) {
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
