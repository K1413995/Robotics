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
constexpr float GOAL_X = 120; // in front of the goal, facing heading 0
constexpr float GOAL_Y = -75;
constexpr float LOAD_ROW_Y = -150; // the row in line with the match load
constexpr float LOAD_X = 160;      // at the match load, facing our wall
// Drives straight out to here before the first turn so it clears the match
// load it starts touching, in cm
constexpr float START_CLEAR_X = 120;
constexpr float START_ROW_Y = -150; // stays on this row on the way out

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
// The intake keeps running this long into the drive out, in milliseconds
constexpr std::uint32_t LOAD_INTAKE_PULL_TIME = 1500;

// CHAINING: moves that only set up the next one don't stop and settle.
// They exit early while still moving, and the next move starts right away.
// More speed or range is faster but less exact. Moves into the goal still stop
constexpr lemlib::MoveToPointParams CHAIN_DRIVE{.minSpeed = 40,
                                               .earlyExitRange = cm(5)};
constexpr lemlib::MoveToPointParams CHAIN_BACK{
    .forwards = false, .minSpeed = 40, .earlyExitRange = cm(5)};
// For turns the range is in degrees. The drive after it steers the rest
constexpr lemlib::TurnToHeadingParams CHAIN_TURN{.minSpeed = 30,
                                                 .earlyExitRange = 5};

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
  pointTo(GOAL_X, LOAD_ROW_Y, 3000, CHAIN_BACK);
  return finished();
}

// From the load row: turn to the wall, drive up, and intake with the lift
// all the way down, pushing so it stays against the wall.
// The intake is left running, carryToGoal stops it
bool loadFromWall() {
  Components::liftTo(LOAD_LIFT);
  turnTo(90, 1500, CHAIN_TURN);
  if (!finished()) {
    return false;
  }
  // The push after this presses it the rest of the way
  pointTo(LOAD_X, LOAD_ROW_Y, 2000, CHAIN_DRIVE);
  if (!finished() || !liftNear(LOAD_LIFT)) {
    return false;
  }
  Components::clawIntake(true);
  return pushForward(LOAD_PUSH_POWER, LOAD_TIME);
}

// The reverse of the trip out: back to the row, turn, raise, drive to the goal.
// Only starts the last drive, scoreAndBackOff finishes it
bool carryToGoal(double carryAngle) {
  pointTo(GOAL_X, LOAD_ROW_Y, 2000, CHAIN_BACK);
  // Keeps pulling the stack in while driving away, then brakes so HOLD
  // keeps the claw closed. This runs on its own, the moves don't wait for it
  after(LOAD_INTAKE_PULL_TIME, [] { Components::clawIntake(false); });
  // Lifts the stack off the floor once it has pulled away from the load
  if (!wait(LOAD_RAISE_DELAY)) {
    return false;
  }
  Components::liftTo(LOAD_LIFT + LOAD_PULL_RAISE);
  if (!finished()) {
    return false;
  }
  turnTo(0, 1500, CHAIN_TURN);
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
  pointTo(START_CLEAR_X, START_ROW_Y, 1500, CHAIN_DRIVE);
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
