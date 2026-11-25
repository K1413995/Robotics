// CLAW
// Intake closes the claw while held. Open runs full power for a set time.
// Otherwise the motor brakes, and the brake mode is HOLD (set in main.cc).
// A small task times the open, so it also works in auton.
#include "components.hpp"
#include "pros/rtos.hpp"

#include <atomic>
#include <cstdint>

namespace {
// Claw power while the intake button is held, out of 127
constexpr int CLAW_INTAKE_POWER = 60;
// Open: full power toward open for this long, then brake
constexpr int CLAW_OPEN_POWER = -127;
constexpr std::uint32_t CLAW_OPEN_TIME = 2000; // milliseconds
constexpr std::uint32_t CLAW_LOOP_MS = 10;

// Shared between the driver or auton code and the claw task
std::atomic<bool> intakeHeld{false};
std::atomic<bool> openRequested{false};
std::atomic<bool> opening{false};

void runClaw() {
  std::uint32_t openStart = 0;
  while (true) {
    const std::uint32_t now = pros::millis();
    if (openRequested.exchange(false)) {
      opening = true;
      openStart = now;
    }

    if (intakeHeld.load()) {
      opening = false; // intake takes over from an open
      Hardware::claw.move(CLAW_INTAKE_POWER);
    } else if (opening.load() && now - openStart < CLAW_OPEN_TIME) {
      Hardware::claw.move(CLAW_OPEN_POWER);
    } else {
      opening = false;
      Hardware::claw.brake();
    }
    pros::delay(CLAW_LOOP_MS);
  }
}
} // namespace

namespace Components {
void startClaw() { static pros::Task clawTask(runClaw, "Claw"); }

// Closes while true, brakes once false
void clawIntake(bool in) { intakeHeld = in; }

// Opens at full power for CLAW_OPEN_TIME, then brakes
void clawOpen() { openRequested = true; }

// True while the claw is still opening
bool clawOpening() { return openRequested.load() || opening.load(); }
} // namespace Components
