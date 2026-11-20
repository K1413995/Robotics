#include "../components/components.hpp"
#include "../lemlib/drivetrain.hpp"
#include "../hardware/hardware.hpp"

void trig() {
    
}

void auton() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(20, 20, 5000);
}