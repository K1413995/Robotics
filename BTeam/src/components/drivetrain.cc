// Drivetrain
// PID values are in pid/pidconfig.hpp
#include "../pid/pidconfig.hpp"
#include "components.hpp"

// Horizontal drift limits speed through curves in moveToPose
// LemLib recommends 8 with traction wheels, 2 with all omnis
// Lower it if the robot slides wide, raise it if curves are too slow
constexpr float HORIZONTAL_DRIFT = 8;

lemlib::Drivetrain drivetrain(&Hardware::leftMotors, &Hardware::rightMotors,
                              11.5, lemlib::Omniwheel::NEW_325, 450,
                              HORIZONTAL_DRIFT);

// Must stay in this file, above chassis. Chassis copies these when it is
// created, and C++ only guarantees that order within a single file
static lemlib::OdomSensors sensors(&Hardware::vertical, // vertical wheel
                                   nullptr, // second vertical wheel
                                   nullptr, // horizontal wheel
                                   nullptr, // second horizontal wheel
                                   &Hardware::imu);

static lemlib::ControllerSettings makeController(const PidConfig::Motion &pid) {
  return lemlib::ControllerSettings(pid.kP, pid.kI, pid.kD, pid.windup,
                                    pid.smallError, pid.smallErrorTimeout,
                                    pid.largeError, pid.largeErrorTimeout,
                                    pid.slew);
}

// Chassis (commbines all above)
lemlib::Chassis chassis(drivetrain, makeController(PidConfig::DRIVE),
                        makeController(PidConfig::TURN), sensors,
                        &Hardware::throttleCurve, &Hardware::steerCurve);
