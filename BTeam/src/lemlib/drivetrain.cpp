// Drivetrain
#include "drivetrain.hpp"

// PID
// lateral PID controller (NEED TO UNDERSTAND)
lemlib::ControllerSettings lateralController(
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
lemlib::ControllerSettings angularController(
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

lemlib::Drivetrain drivetrain(
    &Hardware::leftMotors,
    &Hardware::rightMotors,
    11.5,
    lemlib::Omniwheel::NEW_325,
    450,
    2
);

// Chassis (commbines all above)
lemlib::Chassis chassis(
    drivetrain,
    lateralController,
    angularController,
    Hardware::sensors,
    &Hardware::throttleCurve,
    &Hardware::steerCurve
);
