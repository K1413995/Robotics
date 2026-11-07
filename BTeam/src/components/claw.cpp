#include "components.hpp"

namespace Components {
    void openClaw(bool open) {
        Hardware::openClaw.set_value(open);
    }

    bool lastPositionUp = true;
    void clawUp(bool goUp) {
        if(goUp && !lastPositionUp) {
            openClaw(true);
            Hardware::flipClaw.move_absolute(0, 200);
        }
        else if(!goUp && lastPositionUp) {
            Hardware::flipClaw.move_absolute(280, 200);
            openClaw(false);
        }
        lastPositionUp = goUp;
    }
}
