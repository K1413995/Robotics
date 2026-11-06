#include "components.hpp"

void Components::claw(bool open) {
    Hardware::openClaw.set_value(open);
}