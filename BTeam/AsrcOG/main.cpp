#include "main.h"
#include <cmath>
#include "robot.hpp"

//
void on_center_button() {
	//
}

// Globals
bool stopLift = false;

void initialize() {
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors
    lift.set_brake_mode_all(pros::E_MOTOR_BRAKE_HOLD); // allow lift to be stationary

    // print position to brain screen
    pros::Task screen([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Degree: %d", lift.get_position());
            
            // delay to save resources
            pros::delay(20);
        }
    });

    pros::Task macro([](){
        int targetHeading = 0;
        while (true) {
            bool newRight = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R2);
            bool newLeft = controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_R1);
            float heading = chassis.getPose().theta;
            if (newRight || newLeft) {
                if (chassis.isInMotion()) {
                    if (newRight) {targetHeading += 90;}
                    if (newLeft) {targetHeading -= 90;}
                } else {
                    if (newRight) {
                        targetHeading = (int)(std::floor(heading / 90) + 1) * 90;
                    }
                    if (newLeft) {
                        targetHeading = (int)(std::ceil(heading / 90) - 1) * 90;
                    }
                }
                targetHeading = ((targetHeading % 360) + 360) % 360; // Normalize
            chassis.cancelMotion();
            chassis.turnToHeading(targetHeading, 2000);
            }
            pros::lcd::print(5, "Heading: %.1f Target: %d Moving: %d", heading, targetHeading, chassis.isInMotion());
            pros::delay(20);
        }
    });

    pros::Task liftEfficieny([](){
        int time = 0;
        while (true) {
            bool liftActive = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1) || controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
            bool liftRelease = controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_L1) || controller.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_L2);
            bool efficient = lift.get_efficiency_all()[0] > 10;
            if (liftRelease) {
                time = 0;
                stopLift = false;
            } else if (liftActive && !stopLift) {
                time += 20;
                if (time >= 100 && !efficient) {
                    stopLift = true;
                }
            } else if (!liftActive) {
                time = 0;
            }
            
            pros::lcd::print(4, "Eff: %.1f%% Active: %d Stop: %d", lift.get_efficiency_all()[0], liftActive, stopLift);
            pros::delay(20);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
    chassis.setPose(0, 0, 0);
    //chassis.arcade(0, 50);
}

// test
bool rotateClaw = true;

void opcontrol() {
    // loop forever
    double shift = 1;
    while (true) {
        // Controls
        // get left y and right x
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        // get up or down active
        bool liftUp = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool liftDown = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        // claw
        bool openClaw = !controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {rotateClaw = !rotateClaw;}

        // toggle
        bool toggleActivate = controller.get_digital(pros::E_CONTROLLER_DIGITAL_Y);

        // Shifter
        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP) && shift < 1) {
            shift += .1;
        } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN) && shift > 0.2) {
            shift -= .1;
        }

        // move robot and cancel
        // if (abs(leftY) > 0 || abs(rightX) > 0) {
        //     chassis.cancelAllMotions();
        //     chassis.arcade(leftY, rightX);
        // }
        chassis.arcade((leftY * shift), (rightX * shift));

        // move lift
        // if (!stopLift) {
        //     if (liftUp) { // && (lift.get_position() < 680)
        //         lift.move(50);
        //     } else 
        //     if (liftDown) { // && (lift.get_position() > 0)
        //         lift.move(-50);
        //     } else {lift.brake();}
        // }
        if (liftUp) {
            lift.move(60);
        } else if (liftDown) {
            lift.move(-60);
        } else {
            lift.brake();
        }

        // claw stuf
        openGate.set_value(openClaw);
        flipGate.set_value(rotateClaw);

        //toggle
        if (toggleActivate) {
            toggle.move(127);
        } else {toggle.brake();}

        // delay to save resources
        pros::delay(20);
    }

}