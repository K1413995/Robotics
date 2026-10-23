#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"
#include "robot.hpp"

namespace Ports {
    constexpr int IMU = 10;
    constexpr int VERTICAL = 17;
    constexpr int HORIZONTAL = 18;

    constexpr int LEFT_FRONT = -2;
    constexpr int LEFT_BACK = -3;
    constexpr int RIGHT_FRONT = 11;
    constexpr int RIGHT_BACK = 12;

    constexpr int LIFT_LEFT = 13;
    constexpr int LIFT_RIGHT = -20;

    constexpr int TOGGLE = 8;

    constexpr char OPEN_GATE = 'A';
    constexpr char FLIP_GATE = 'B';
}

namespace Settings {
    constexpr int LIFT_MAX = 680;
    constexpr int LIFT_SPEED_UP = 60;
    constexpr int LIFT_SPEED_DOWN = -60;
}

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

//Sensors
pros::IMU imu(Ports::IMU);

pros::Rotation vertical_sensor(Ports::VERTICAL);
pros::Rotation horizontal_sensor(Ports::HORIZONTAL);

lemlib::TrackingWheel vertical(
    &vertical_sensor,
    lemlib::Omniwheel::NEW_275,
    0
);

lemlib::TrackingWheel horizontal(
    &horizontal_sensor,
    lemlib::Omniwheel::NEW_275,
    0
);

lemlib::OdomSensors sensors(
    &vertical,   // vertical tracking wheel
    nullptr,     // second vertical wheel
    &horizontal, // horizontal tracking wheel
    nullptr,     // second horizontal wheel
    &imu
);

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

// Motor Groups
pros::MotorGroup leftMotors({Ports::LEFT_FRONT, Ports::LEFT_BACK}, pros::v5::MotorGearset::blue);
pros::MotorGroup rightMotors({Ports::RIGHT_FRONT, Ports::RIGHT_BACK}, pros::v5::MotorGearset::blue);

pros::MotorGroup lift({Ports::LIFT_LEFT, Ports::LIFT_RIGHT}, pros::v5::MotorGearset::green, pros::MotorEncoderUnits::deg);

pros::adi::DigitalOut openGate(Ports::OPEN_GATE, false);
pros::adi::DigitalOut flipGate(Ports::FLIP_GATE, false);

pros::Motor toggle(Ports::TOGGLE, pros::v5::MotorGearset::green);

// Drivetrain
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
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
