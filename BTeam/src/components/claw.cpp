#include "components.hpp"

namespace Components {
    void openClaw(bool open) {
        Hardware::openClaw.set_value(open);
    }

    bool lastPositionUp = true;
    void clawUp(bool goUp) {
        if(goUp && !lastPositionUp) {Hardware::flipClaw.move_relative(90, 100);}
        else if(!goUp && lastPositionUp) {Hardware::flipClaw.move_relative(-90, 100);}
        lastPositionUp = goUp;
    }
}
