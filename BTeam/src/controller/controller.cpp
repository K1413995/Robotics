#include "controller.hpp"
#include "../components/components.hpp"
#include "../lemlib/drivetrain.hpp"
#include <cmath>

//globals

namespace Controller {

    // Auton start
    bool auton = false;
    // Current controller readings
    int drive = 0;
    int steer = 0;

    bool liftUp = false;
    bool liftDown = false;

    bool clawTap = false;
    bool clawIntake = false;

    bool toggleOnce = false;

    // Updates controller values
    void update(const Controls& driver) {
        //auton = Hardware::controller.get_digital_new_press(driver.auton);

        drive = Hardware::controller.get_analog(driver.leftY);
        steer = Hardware::controller.get_analog(driver.rightX);

        liftUp = Hardware::controller.get_digital(driver.liftUp);
        liftDown = Hardware::controller.get_digital(driver.liftDown);

        clawIntake = Hardware::controller.get_digital(driver.claw);
        clawTap = Hardware::controller.get_digital_new_press(driver.claw);

        toggleOnce = Hardware::controller.get_digital_new_press(driver.toggle);
    }

    // Looped logic
    void execute() {
        //drive
        chassis.arcade(drive, steer);
        //lift
        if (liftUp) {Components::lift(95);}
        else if (liftDown) {Components::lift(-95);}
        else {Components::lift(0);}
        //claw
        Components::clawIntake(clawIntake);
        Components::clawOpen(clawTap); //flip input
        //toggle
        Components::toggle(toggleOnce);
    }
}