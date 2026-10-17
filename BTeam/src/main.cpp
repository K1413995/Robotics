#include "main.h"
#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include <cmath>

// lateral PID controller (NEED TO UNDERSTAND)
lemlib::ControllerSettings lateral_controller(
    10, // proportional gain (kP)
    0, // integral gain (kI)
    3, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in inches
    100, // small error range timeout, in milliseconds
    3, // large error range, in inches
    500, // large error range timeout, in milliseconds
    20 // maximum acceleration (slew)
);

// angular PID controller (NEED TO UNDERSTAND)
lemlib::ControllerSettings angular_controller(
    2, // proportional gain (kP)
    0, // integral gain (kI)
    10, // derivative gain (kD)
    3, // anti windup
    1, // small error range, in degrees
    100, // small error range timeout, in milliseconds
    3, // large error range, in degrees
    500, // large error range timeout, in milliseconds
    0 // maximum acceleration (slew)
);

// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttle_curve(
    3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steer_curve(
    3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Sensors
pros::IMU imu(10);
//tracking wheel x
//tracking wheel y

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

// Motor Groups
pros::MotorGroup left_motors({-2, -3}, pros::v5::MotorGearset::blue);
pros::MotorGroup right_motors({11, 12}, pros::v5::MotorGearset::blue);

pros::MotorGroup lift({13, -20}, pros::v5::MotorGearset::green, pros::MotorEncoderUnits::deg);

pros::adi::DigitalOut openGate('A', false);
pros::adi::DigitalOut flipGate('B', false);

pros::Motor toggle(8, pros::v5::MotorGearset::green);

// Drivetrain
lemlib::Drivetrain drivetrain(
    &left_motors,
    &right_motors,
    11.5,
    lemlib::Omniwheel::NEW_325,
    450,
    2
);

// Chassis (commbines all above)
lemlib::Chassis chassis(
    drivetrain,
    lateral_controller,
    angular_controller,
    sensors,
    &throttle_curve,
    &steer_curve
);

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
}

// test
bool rotateClaw = true;

void opcontrol() {
    // loop forever
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

        // move robot and cancel
        // if (abs(leftY) > 0 || abs(rightX) > 0) {
        //     chassis.cancelAllMotions();
        //     chassis.arcade(leftY, rightX);
        // }
        chassis.arcade(leftY, rightX);

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
            lift.move(50);
        } else if (liftDown) {
            lift.move(-50);
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