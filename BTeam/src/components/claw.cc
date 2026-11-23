#include "components.hpp"

namespace Components {
// Claw power while the intake button is held, out of 127
constexpr int CLAW_INTAKE_POWER = 60;
// Power left on after intake so the ratchet keeps pulling the claw closed
// 30 is about 2.8 volts, and 127 is the full 12 volts
// Raise it if the claw doesn't turn, lower it if the claw motor gets hot
constexpr int CLAW_IDLE_POWER = 30;

bool idling = false;
bool clawIsOpen = false;
bool hasClosedPosition = false;
double closedPosition = 0;

void clawIntake(bool in) {
  if (in) {
    Hardware::claw.move(CLAW_INTAKE_POWER);
    idling = true;
    clawIsOpen = false;
    // Intake moved the claw, so find a new reset spot
    hasClosedPosition = false;
  } else if (idling) {
    // Sent every loop until clawOpen turns it off
    Hardware::claw.move(CLAW_IDLE_POWER);
  }
}

// R1 switches between open and the closed reset position
void clawOpen(bool open) {
  if (open) {
    idling = false;
    if (clawIsOpen) {
      Hardware::claw.move_absolute(closedPosition, 200);
      clawIsOpen = false;
    } else {
      if (!hasClosedPosition) {
        closedPosition = Hardware::claw.get_position();
        hasClosedPosition = true;
      }
      Hardware::claw.move_absolute(closedPosition - 220, 200);
      clawIsOpen = true;
    }
  }
}
} // namespace Components
