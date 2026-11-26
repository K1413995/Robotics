#pragma once

#include "../components/components.hpp"
#include "../interface/settings.hpp"

#include <cstdint>
#include <functional>

// AUTON HELPERS
// Points are LEFT side field coordinates in centimeters (Jerry.io), and flip
// for the right side. See the field plane in autonconfig.hpp
// After every move use: if (!finished()) return;
namespace Auton {

// Jerry.io shows centimeters by default, LemLib uses inches
constexpr float cm(float centimeters) { return centimeters / 2.54f; }

// Call first in every auton, picks which side the points flip for
void begin(Settings::Side side);

// Mirroring flips left/right (Y) only. X, toward the enemy, stays the same
float sideY(float y);
// 0 and 180 swap, 90 (our wall) and 270 (the enemy) stay the same
float sideHeading(float heading);

// MOVES, these start the move and return right away
// Drives to a point and ends facing a heading, on a curve
void moveTo(float xCm, float yCm, float heading, int timeout,
            lemlib::MoveToPoseParams params = {});
// Drives to a point in a straight line, set .forwards = false to back up
void pointTo(float xCm, float yCm, int timeout,
             lemlib::MoveToPointParams params = {});
// Turns in place to a heading
void turnTo(float heading, int timeout,
            lemlib::TurnToHeadingParams params = {});
// Straight along the way the robot faces now: + forward, - backward, in cm
void driveStraight(float distanceCm, int timeout,
                   lemlib::MoveToPointParams params = {});

// WAITING, each returns false if the test code canceled the auton
// Waits for the current move to end
bool finished();
// Waits for the lift to reach its target
bool liftFinished(std::uint32_t timeoutMs = 1000);
// Waits until the lift is within toleranceDeg of an angle. Quicker than
// liftFinished, which also waits for the PID's slow last degree or two
bool liftNear(double angle, double toleranceDeg = 2,
              std::uint32_t timeoutMs = 1000);
// Waits until the robot is within rangeCm of a point, or the move ends.
// Lets the next step start while the move is still finishing
bool waitUntilNear(float xCm, float yCm, float rangeCm);
// A plain pause
bool wait(std::uint32_t ms);
// Drives forward at a steady power, like pushing into the wall, then stops
bool pushForward(int power, std::uint32_t ms);

// Runs an action ms from now on its own, while the moves carry on.
// Skipped if the test code cancels the auton first
void after(std::uint32_t ms, std::function<void()> action);

} // namespace Auton
