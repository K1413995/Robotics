// HARDWARE
#include "hardware.hpp"
#include "pros/abstract_motor.hpp"

namespace Hardware {

// Controller
pros::Controller controller(pros::E_CONTROLLER_MASTER);

// Motor Groups
pros::MotorGroup liftMotors({Ports::LIFT_LEFT, Ports::LIFT_RIGHT},
                            pros::v5::MotorGearset::green,
                            pros::MotorEncoderUnits::deg);

pros::Motor claw(Ports::CLAW, pros::v5::MotorGearset::green);

pros::MotorGroup leftMotors({Ports::LEFT_FRONT, Ports::LEFT_BACK},
                            pros::v5::MotorGearset::blue);
pros::MotorGroup rightMotors({Ports::RIGHT_FRONT, Ports::RIGHT_BACK},
                             pros::v5::MotorGearset::blue);

// Sensors
pros::IMU imu(Ports::IMU);

pros::Rotation verticalSensor(Ports::VERTICAL);
pros::Rotation liftRotation(Ports::LIFT_ROTATION);

pros::Distance leftDistance(Ports::LEFT_DISTANCE);
pros::Distance rightDistance(Ports::RIGHT_DISTANCE);
pros::Distance backDistance(Ports::BACK_DISTANCE);

lemlib::TrackingWheel vertical(&verticalSensor, lemlib::Omniwheel::NEW_275, 0);

// Controller config
// input curve for throttle input during driver control
lemlib::ExpoDriveCurve throttleCurve(
    3, // joystick deadband out of 127
    0, // 10, // minimum output where drivetrain will move out of 127
    1  // 1.019 // expo curve gain
);

// input curve for steer input during driver control
lemlib::ExpoDriveCurve
    steerCurve(3, // joystick deadband out of 127
               0, // 10, // minimum output where drivetrain will move out of 127
               1  // 1.019 // expo curve gain
    );
} // namespace Hardware
