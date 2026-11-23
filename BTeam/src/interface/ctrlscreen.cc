// CONTROLLER SCREEN
#include "ctrlscreen.hpp"
#include "../hardware/hardware.hpp"
#include "pros/rtos.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>

namespace CtrlScreen {
namespace {

// The controller screen is slow, so space out updates
constexpr std::uint32_t SCREEN_DELAY_MS = 60;
constexpr int SCREEN_LINES = 3;
constexpr int SCREEN_WIDTH = 15;

char screen[SCREEN_LINES][SCREEN_WIDTH + 1] = {};
bool lineChanged[SCREEN_LINES] = {};
const char *pendingRumble = nullptr;
std::uint32_t lastSend = 0;

} // namespace

void write(int line, const char *text) {
  char next[SCREEN_WIDTH + 1];
  std::snprintf(next, sizeof(next), "%-*s", SCREEN_WIDTH, text);
  if (std::strcmp(next, screen[line]) != 0) {
    std::strcpy(screen[line], next);
    lineChanged[line] = true;
  }
}

void rumble(const char *pattern) { pendingRumble = pattern; }

void update() {
  const std::uint32_t now = pros::millis();
  if (now - lastSend < SCREEN_DELAY_MS) {
    return;
  }
  if (pendingRumble != nullptr) {
    Hardware::controller.rumble(pendingRumble);
    pendingRumble = nullptr;
    lastSend = now;
    return;
  }
  for (int line = 0; line < SCREEN_LINES; line++) {
    if (lineChanged[line]) {
      if (Hardware::controller.set_text(line, 0, screen[line]) != PROS_ERR) {
        lineChanged[line] = false;
      }
      lastSend = now;
      return;
    }
  }
}

} // namespace CtrlScreen
