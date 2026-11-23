#pragma once

#include <cstdint>

// LIFT TUNING
// Limits and driver feel. Speeds are degrees per second at the lift.
// The lift's PID values are in pid/pidconfig.hpp
namespace LiftConfig {

// LIMITS, in degrees, using the angle shown on the brain screen
constexpr double MIN_ANGLE = 5;
constexpr double MAX_ANGLE = 200;
// Use -1 if the angle drops as the lift rises
constexpr int ANGLE_DIRECTION = 1;

// GEARING
// Green cartridge top speed in RPM, and the gears from the motor to the lift
constexpr double MOTOR_RPM = 200;
constexpr double GEAR_RATIO = 12.0 / 36.0;
// Lift top speed at power 127 (1 RPM = 6 deg/s)
constexpr double FULL_SPEED = MOTOR_RPM * GEAR_RATIO * 6;
// The lift task needs a steady loop, in milliseconds
constexpr std::uint32_t LOOP_MS = 10;

// DRIVER: HOW LONG THE BUTTON IS HELD
// A short press stays slow for accuracy, a long hold builds to full speed
constexpr double START_SPEED = 20; // speed a press starts from
constexpr double TAP_SPEED = 50;   // speed reached by the end of a tap
constexpr double TAP_TIME = 100;   // milliseconds a press counts as a tap
constexpr double ACCEL_TIME = 250; // milliseconds from tap speed to full speed
// Most the speed can change while driving, in degrees per second^2
constexpr double MAX_SPEED_CHANGE = 3500;

// DRIVER: POSITION, slowing down for the top and bottom limits
// How hard it may slow down, in degrees per second^2
// Lower starts slowing earlier and lands softer
constexpr double BOTTOM_DECEL = 1600;
constexpr double TOP_DECEL = 3000;
// Speed the lift reaches a limit at
constexpr double LANDING_SPEED = 10;
// How gently the last part of a stop fades out, lower is softer and longer
constexpr double BOTTOM_EASE = 8;
constexpr double TOP_EASE = 20;
// Seconds of travel to look ahead, covers the motors' reaction time
constexpr double LOOK_AHEAD = 0.06;

// DRIVER: LETTING GO
// Faster lifts get a longer fade, so taps stop right away and full speed stops
// without bouncing. Lower is softer but travels farther after release
constexpr double RELEASE_DECEL = 4500; // degrees per second^2
constexpr double RELEASE_MIN_TIME = 40; // shortest fade, in milliseconds
// Milliseconds to settle at zero speed before HOLD locks in
constexpr double RELEASE_SETTLE = 100;

// FLOOR AND STALL PROTECTION
// The driver's floor check only runs this close to the bottom, in degrees
constexpr double FLOOR_ZONE = 12;
// If the lift moves less than this many degrees...
constexpr double STALL_MOVEMENT = 0.4;
// ...for this many milliseconds, it is blocked and the motors stop pushing
constexpr std::uint32_t STALL_TIME = 150;
// Milliseconds a new press gets to start moving before the floor check runs
constexpr std::uint32_t STALL_GRACE = 80;

} // namespace LiftConfig
