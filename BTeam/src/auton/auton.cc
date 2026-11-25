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
constexpr float LOAD_ROW_Y = -145; // the row in line with the match load
constexpr float LOAD_X = 160;      // at the match load, facing our wall
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
// After intaking, the lift rises this many degrees to carry the stack
constexpr double LOAD_PULL_RAISE = 5;
// Milliseconds after it starts pulling out, 0 lifts right away
constexpr std::uint32_t LOAD_RAISE_DELAY = 500;

// Scoring
// The lift starts lowering onto the goal this close to it, in cm, while the
// drive is still settling. 0 waits for the drive to fully stop
constexpr float SCORE_LOWER_RANGE = 5;
// The claw opens once the lift is this close to the score angle, in degrees
constexpr double SCORE_LIFT_TOLERANCE = 2;

// Match load
constexpr int LOAD_PUSH_POWER = 25; // keeps it pressed on the wall, out of 127
constexpr std::uint32_t LOAD_TIME = 1500; // milliseconds intaking

// Call right after starting the drive to the goal. Lowers once close, lets go,
// and backs straight out to the load row without waiting for the claw to open
bool scoreAndBackOff(double scoreAngle) {
  if (!waitUntilNear(GOAL_X, GOAL_Y, SCORE_LOWER_RANGE)) {
    return false;
  }
  Components::liftTo(scoreAngle);
  if (!finished() || !liftNear(scoreAngle, SCORE_LIFT_TOLERANCE)) {
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

// The reverse of the trip out: back to the row, turn, raise, drive to the goal.
// Only starts the last drive, scoreAndBackOff finishes it
bool carryToGoal(double carryAngle) {
  pointTo(GOAL_X, LOAD_ROW_Y, 2000, {.forwards = false});
  // Lifts the stack off the floor once it has pulled away from the load
  if (!wait(LOAD_RAISE_DELAY)) {
    return false;
  }
  Components::liftTo(LOAD_LIFT + LOAD_PULL_RAISE);
  if (!finished()) {
    return false;
  }
  turnTo(0, 1500);
  if (!finished()) {
    return false;
  }
  pointTo(GOAL_X, GOAL_Y, 3000);
  Components::liftTo(carryAngle);
  return true;
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
