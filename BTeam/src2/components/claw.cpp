#include "components.hpp"

void claw(bool open) {
    Hardware::openClaw.set_value(open);
}