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

    bool openClaw = true;
    bool openClawRelease = false;
    bool flipClaw = false;

    bool toggleOn = false;

    // Updates controller values
    void update(const Controls& driver) {
        //auton = Hardware::controller.get_digital_new_press(driver.auton);

        drive = Hardware::controller.get_analog(driver.leftY);
        steer = Hardware::controller.get_analog(driver.rightX);

        liftUp = Hardware::controller.get_digital(driver.liftUp);
        liftDown = Hardware::controller.get_digital(driver.liftDown);

        openClaw = Hardware::controller.get_digital(driver.openClaw);
        openClawRelease = Hardware::controller.get_digital_new_release(driver.openClaw);
        if ( //toggles mode
            Hardware::controller.get_digital_new_press(driver.flipClaw)
        ) {flipClaw = !flipClaw;}

        toggleOn = Hardware::controller.get_digital(driver.toggle);
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
        Components::openClaw(!openClaw); //flip input
        if(openClawRelease) {
            Hardware::liftMotors.move_relative(120, 200);
            //maybe add delayy to prevent john prefire move
        }
        Components::clawUp(flipClaw);
        //toggle
        Components::toggle(toggleOn);
    }
}