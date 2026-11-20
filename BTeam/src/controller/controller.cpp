#include "controller.hpp"
#include "../components/components.hpp"
#include "../lemlib/drivetrain.hpp"
#include "../hardware/hardware.hpp"

namespace Controller {

    void execute(const Controls& driver) {
        //drive
        int drive = Hardware::controller.get_analog(driver.leftY);
        int steer = Hardware::controller.get_analog(driver.rightX) * .95; //reduce by 5% to compensate

        chassis.arcade(drive, steer);

        //lift
        if (driver.liftUp.pressed()) {
            Components::lift(95);
        }
        else if (driver.liftDown.pressed()) {
            Components::lift(-95);
        }
        else {
            Components::lift(0);
        }

        //claw
        Components::clawOpen(driver.clawOpen.tapped());
        Components::clawIntake(driver.clawIntake.held());

        //toggle
        Components::toggle(driver.toggle.newPress());
    }
}