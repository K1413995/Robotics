#pragma once

#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"

namespace Ports {
    constexpr int IMU = 2;
    constexpr int VERTICAL = 6;
    constexpr int HORIZONTAL = 9;

    constexpr int LEFT_FRONT = -11;
    constexpr int LEFT_BACK = -12;
    constexpr int RIGHT_FRONT = 20;
    constexpr int RIGHT_BACK = 18;

    constexpr int LIFT_LEFT = 5;
    constexpr int LIFT_RIGHT = 7;

    constexpr int TOGGLE = 8;

    constexpr bool CLAW = -19;
}

namespace Hardware {
    // Controller
    extern pros::Controller controller;

    // Motor Groups
    extern pros::MotorGroup liftMotors;
    extern pros::MotorGroup leftMotors;
    extern pros::MotorGroup rightMotors;

    extern pros::Motor toggle;

    extern pros::Motor claw;

    // Sensors
    extern pros::IMU imu;

    extern pros::Rotation verticalSensor;

    // LemLib tracking wheels
    extern lemlib::TrackingWheel vertical;

    extern lemlib::OdomSensors sensors;

    // LemLib controllers
    extern lemlib::ControllerSettings lateralController;
    extern lemlib::ControllerSettings angularController;

    // Driver curves
    extern lemlib::ExpoDriveCurve throttleCurve;
    extern lemlib::ExpoDriveCurve steerCurve;
}