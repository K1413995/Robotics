#include "controller.hpp"
#include "../lemlib/drivetrain.hpp"
#include <cmath>

//globals

namespace Controller {

    // Current controller readings
    int leftY = 0;
    int rightX = 0;

    bool liftUp = false;
    bool liftDown = false;

    bool openClaw = false;
    bool toggle = false;

    // Updates controller values
    void update(const Controls& driver) {
        leftY = Hardware::controller.get_analog(driver.leftY);
        rightX = Hardware::controller.get_analog(driver.rightX);

        liftUp = Hardware::controller.get_digital(driver.liftUp);
        LiftDown = Hardware::controller.get_digital(driver.liftDown);

        openClaw = Hardware::controller.get_digital(driver.openClaw);

        toggle = Hardware::controller.get_digital_new_press(driver.toggle);
    }

    // Looped logic
    void act() {
        //this will handle logic
        chassis.arcade(leftY, rightX);
    }
}