#pragma once

#include "lemlib/api.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/distance.hpp"
#include "pros/misc.h"
#include "pros/rotation.hpp"

namespace Ports {
constexpr int IMU = 6;
constexpr int VERTICAL = 12;
constexpr int LEFT_DISTANCE = 14;
constexpr int BACK_DISTANCE = 17;
constexpr int RIGHT_DISTANCE = 19;

constexpr int LEFT_FRONT = -3;
constexpr int LEFT_BACK = -11;
constexpr int RIGHT_FRONT = 8;
constexpr int RIGHT_BACK = 20;

constexpr int LIFT_LEFT = -5;
constexpr int LIFT_RIGHT = 7;
constexpr int LIFT_ROTATION = 4;

constexpr int CLAW = -18;
} // namespace Ports

namespace Hardware {
// Controller
extern pros::Controller controller;

// Motor Groups
extern pros::MotorGroup liftMotors;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern pros::Motor claw;

// Sensors
extern pros::IMU imu;
// extern pros::Rotation verticalSensor;
extern pros::Rotation liftRotation;
extern pros::Distance leftDistance;
extern pros::Distance rightDistance;
extern pros::Distance backDistance;

// LemLib tracking wheels
extern lemlib::TrackingWheel vertical;

// Driver curves
extern lemlib::ExpoDriveCurve throttleCurve;
extern lemlib::ExpoDriveCurve steerCurve;
} // namespace Hardware
