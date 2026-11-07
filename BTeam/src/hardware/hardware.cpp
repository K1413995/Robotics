// HARDWARE
#include "hardware.hpp"

namespace Hardware {

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
pros::MotorGroup liftMotors({Ports::LIFT_LEFT, Ports::LIFT_RIGHT}, pros::v5::MotorGearset::green, pros::MotorEncoderUnits::deg);

pros::adi::DigitalOut claw(Ports::OPEN_CLAW, false);
pros::Motor flipClaw(Ports::FLIP_CLAW, pros::v5::MotorGearset::rpm_200);

pros::Motor toggle(Ports::TOGGLE, pros::v5::MotorGearset::rpm_200);

pros::MotorGroup leftMotors({Ports::LEFT_FRONT, Ports::LEFT_BACK}, pros::v5::MotorGearset::blue);
pros::MotorGroup rightMotors({Ports::RIGHT_FRONT, Ports::RIGHT_BACK}, pros::v5::MotorGearset::blue);

//Sensors
pros::IMU imu(Ports::IMU);

pros::Rotation verticalSensor(Ports::VERTICAL);
pros::Rotation horizontalSensor(Ports::HORIZONTAL);

lemlib::TrackingWheel vertical(
    &verticalSensor,
    lemlib::Omniwheel::NEW_275,
    0
);

lemlib::TrackingWheel horizontal(
    &horizontalSensor,
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

// Controller config
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(
    3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve steerCurve(
    3, // joystick deadband out of 127
    10, // minimum output where drivetrain will move out of 127
    1.019 // expo curve gain
);
}
