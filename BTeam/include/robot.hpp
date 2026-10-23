#pragma once

#include "main.h"
#include "lemlib/api.hpp"

extern pros::Controller controller;
extern pros::IMU imu;

extern pros::Rotation vertical_sensor;
extern pros::Rotation horizontal_sensor;

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::MotorGroup lift;

extern pros::adi::DigitalOut openGate;
extern pros::adi::DigitalOut flipGate;
extern pros::Motor toggle;

extern lemlib::Chassis chassis;