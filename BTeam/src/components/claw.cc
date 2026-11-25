// CLAW
// Intake closes the claw while held. Open turns the claw a set distance.
// Otherwise the motor brakes, and the brake mode is HOLD (set in main.cc).
// A small task runs the open, so it also works in auton.
#include "components.hpp"
#include "pros/rtos.hpp"

#include <atomic>
#include <cmath>
#include <cstdint>

namespace {
// Claw power while the intake button is held, out of 127
constexpr int CLAW_INTAKE_POWER = 60;

// OPEN
// How far to open, in motor degrees: the distance full speed covers in half a
// second. Green motor = 200 rpm = 1200 degrees per second, so 600.
// The real claw is a little slower under load, so lower this if it opens too far
constexpr double CLAW_OPEN_DEGREES = 200 * 6 * 0.5;
constexpr int CLAW_OPEN_DIRECTION = -1;  // negative opens
constexpr int CLAW_OPEN_VELOCITY = 200;  // rpm, full speed for a green motor
constexpr double CLAW_OPEN_TOLERANCE = 10; // degrees from the target counts as done
constexpr std::uint32_t CLAW_OPEN_TIMEOUT = 2000; // milliseconds
// Stuck against something: after this long, slower than STALL_RPM means stop
constexpr std::uint32_t CLAW_STALL_GRACE = 150; // milliseconds
constexpr double CLAW_STALL_RPM = 5;

constexpr std::uint32_t CLAW_LOOP_MS = 10;

// Shared between the driver or auton code and the claw task
std::atomic<bool> intakeHeld{false};
std::atomic<bool> openRequested{false};
std::atomic<bool> opening{false};

void runClaw() {
  std::uint32_t openStart = 0;
  double openTarget = 0;
  while (true) {
    const std::uint32_t now = pros::millis();
    if (openRequested.exchange(false) && !intakeHeld.load()) {
      opening = true;
      openStart = now;
      const double move = CLAW_OPEN_DIRECTION * CLAW_OPEN_DEGREES;
      openTarget = Hardware::claw.get_position() + move;
      Hardware::claw.move_relative(move, CLAW_OPEN_VELOCITY);
    }

    if (intakeHeld.load()) {
      opening = false; // intake takes over from an open
      Hardware::claw.move(CLAW_INTAKE_POWER);
    } else if (opening.load()) {
      const std::uint32_t elapsed = now - openStart;
      const bool arrived = std::fabs(Hardware::claw.get_position() - openTarget) <=
                           CLAW_OPEN_TOLERANCE;
      const bool stalled =
          elapsed >= CLAW_STALL_GRACE &&
          std::fabs(Hardware::claw.get_actual_velocity()) < CLAW_STALL_RPM;
      if (arrived || stalled || elapsed >= CLAW_OPEN_TIMEOUT) {
        opening = false;
        Hardware::claw.brake(); // HOLD keeps it open right here
      }
    } else {
      Hardware::claw.brake(); // also right after the intake stops
    }
    pros::delay(CLAW_LOOP_MS);
  }
}
} // namespace

namespace Components {
void startClaw() { static pros::Task clawTask(runClaw, "Claw"); }

// Closes while true, brakes once false
void clawIntake(bool in) { intakeHeld = in; }

// Opens the claw quickly, then brakes to hold it open
void clawOpen() { openRequested = true; }

// True while the claw is still opening
bool clawOpening() { return openRequested.load() || opening.load(); }
} // namespace Components
