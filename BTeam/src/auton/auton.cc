// AUTONS
// Write points for the LEFT side, in centimeters (Jerry.io). They flip for the
// right side on their own. Moves and waits are in autonhelpers.hpp
// After every move use: if (!finished()) return;
#include "auton.hpp"
#include "../components/liftconfig.hpp"
#include "autonconfig.hpp"
#include "autonhelpers.hpp"
#include "position.hpp"

#include <cstdint>

namespace Auton {
namespace {

// MATCH AUTON SETTINGS
// Points, in cm
constexpr float GOAL_X = 115; // in front of the goal, facing heading 0
constexpr float GOAL_Y = -45;
constexpr float LOAD_ROW_Y = -148; // the row in line with the match load
constexpr float LOAD_X = 145;      // at the match load, facing our wall
// Drives straight this far before the first turn so it clears the match load
// it starts touching, in cm
constexpr float START_CLEAR = 15;

// Lift angles, in brain-screen degrees
constexpr double FIRST_CARRY = 25.5; // raised while driving to the goal
constexpr double FIRST_SCORE = 18;   // lowered onto the goal
// GUESSES for the second stack, which sits on top of the first
constexpr double SECOND_CARRY = 40;
constexpr double SECOND_SCORE = 32.5;
constexpr double LOAD_LIFT = LiftConfig::MIN_ANGLE; // all the way down

// Match load
constexpr int LOAD_PUSH_POWER = 25; // keeps it pressed on the wall, out of 127
constexpr std::uint32_t LOAD_TIME = 1500; // milliseconds intaking

// Lowers onto the goal, lets go, and backs straight out to the load row
// without waiting for the claw to finish opening
bool scoreAndBackOff(double scoreAngle) {
  Components::liftTo(scoreAngle);
  if (!liftFinished()) {
    return false;
  }
  Components::clawOpen();
  pointTo(GOAL_X, LOAD_ROW_Y, 3000, {.forwards = false});
  return finished();
}

// From the load row: turn to the wall, drive up, and intake with the lift
// all the way down, pushing so it stays against the wall
bool loadFromWall() {
  Components::liftTo(LOAD_LIFT);
  turnTo(90, 1500);
  if (!finished()) {
    return false;
  }
  pointTo(LOAD_X, LOAD_ROW_Y, 2000);
  if (!finished() || !liftFinished()) {
    return false;
  }
  Components::clawIntake(true);
  const bool loaded = pushForward(LOAD_PUSH_POWER, LOAD_TIME);
  Components::clawIntake(false); // brakes, HOLD keeps the claw closed
  return loaded;
}

// The reverse of the trip out: back to the row, turn, raise, drive to the goal
bool carryToGoal(double carryAngle) {
  pointTo(GOAL_X, LOAD_ROW_Y, 2000, {.forwards = false});
  if (!finished()) {
    return false;
  }
  turnTo(0, 1500);
  if (!finished()) {
    return false;
  }
  pointTo(GOAL_X, GOAL_Y, 3000);
  Components::liftTo(carryAngle);
  return finished();
}

} // namespace

void match(Settings::Side side) {
  begin(side);

  // Straight out first so turning doesn't clip the match load
  driveStraight(START_CLEAR, 1000);
  if (!finished()) {
    return;
  }

  // To the goal, ending 90 degrees right (heading 0), raising the lift
  moveTo(GOAL_X, GOAL_Y, 0, 3000);
  Components::liftTo(FIRST_CARRY);
  if (!finished()) {
    return;
  }
  if (!scoreAndBackOff(FIRST_SCORE)) {
    return;
  }

  // Second stack from the match load
  if (!loadFromWall()) {
    return;
  }
  if (!carryToGoal(SECOND_CARRY)) {
    return;
  }
  if (!scoreAndBackOff(SECOND_SCORE)) {
    return;
  }
}

void skills(Settings::Side side) {
  begin(side);
  // TODO: skills route
}

void run(Settings::Routine routine, Settings::Side side) {
  if (routine == Settings::Routine::Skills) {
    skills(side);
  } else {
    match(side);
  }
}

bool setStartPose(Settings::Side side) {
  return Position::setPos(AutonConfig::START_HEADING,
                          side == Settings::Side::Right ? "right" : "left");
}

} // namespace Auton
