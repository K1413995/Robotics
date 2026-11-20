#pragma once

#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"

namespace Ports {
    constexpr int IMU = 6;
    constexpr int VERTICAL = 12;
    constexpr int HORIZONTAL = 9;
    constexpr int LEFT_DISTANCE = 0;
    constexpr int BACK_DISTANCE = 0;
    constexpr int RIGHT_DISTANCE = 0;

    constexpr int LEFT_FRONT = -3;
    constexpr int LEFT_BACK = -11;
    constexpr int RIGHT_FRONT = 8;
    constexpr int RIGHT_BACK = 20;

    constexpr int LIFT_LEFT = -5;
    constexpr int LIFT_RIGHT = 7;

    constexpr int TOGGLE = 8;

    constexpr int CLAW = -21;
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
    //extern pros::IMU imu;
    //extern pros::Rotation verticalSensor;
    //extern pros::Distance leftDistance;
    //extern pros::Distance leftDistance;

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