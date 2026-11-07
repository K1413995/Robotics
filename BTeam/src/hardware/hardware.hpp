#pragma once

#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"

namespace Ports {
    constexpr int IMU = 10;
    constexpr int VERTICAL = 17;
    constexpr int HORIZONTAL = 18;

    constexpr int LEFT_FRONT = -15;
    constexpr int LEFT_BACK = -14;
    constexpr int RIGHT_FRONT = 5;
    constexpr int RIGHT_BACK = 7;

    constexpr int LIFT_LEFT = 13;
    constexpr int LIFT_RIGHT = -11;

    constexpr int TOGGLE = 8;

    constexpr char OPEN_CLAW = 'A';
    constexpr int FLIP_CLAW = 17;
}

namespace Hardware {
    // Controller
    extern pros::Controller controller;

    // Motor Groups
    extern pros::MotorGroup liftMotors;
    extern pros::MotorGroup leftMotors;
    extern pros::MotorGroup rightMotors;

    extern pros::Motor toggle;

    extern pros::adi::DigitalOut openClaw;
    extern pros::Motor flipClaw;

    // Sensors
    extern pros::IMU imu;

    extern pros::Rotation verticalSensor;
    extern pros::Rotation horizontalSensor;

    // LemLib tracking wheels
    extern lemlib::TrackingWheel vertical;
    extern lemlib::TrackingWheel horizontal;

    extern lemlib::OdomSensors sensors;

    // LemLib controllers
    extern lemlib::ControllerSettings lateralController;
    extern lemlib::ControllerSettings angularController;

    // Driver curves
    extern lemlib::ExpoDriveCurve throttleCurve;
    extern lemlib::ExpoDriveCurve steerCurve;
}