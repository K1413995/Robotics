// AUTONS
// Each auton is one sequence, top to bottom. Write points for the LEFT side,
// in centimeters (Jerry.io), and they flip for the right side on their own.
// Moves start and return right away, so after each one wait with:
//   if (!finished()) return;
// The moves and waits are explained in autonhelpers.hpp
#include "auton.hpp"
#include "../components/liftconfig.hpp"
#include "autonconfig.hpp"
#include "autonhelpers.hpp"
#include "position.hpp"

namespace Auton {
namespace {

// CHAINING: for moves that only set up the next one. They exit early while
// still moving instead of stopping to settle, so the next move starts sooner.
// More speed or range is faster but less exact
constexpr lemlib::MoveToPointParams CHAIN_DRIVE{.minSpeed = 40,
                                               .earlyExitRange = cm(5)};
constexpr lemlib::MoveToPointParams CHAIN_BACK{
    .forwards = false, .minSpeed = 40, .earlyExitRange = cm(5)};
// For turns the range is in degrees. The drive after it steers the rest
constexpr lemlib::TurnToHeadingParams CHAIN_TURN{.minSpeed = 30,
                                                 .earlyExitRange = 5};

// The lift's lowest angle, also the intake height
constexpr double LIFT_BOTTOM = LiftConfig::MIN_ANGLE;

} // namespace

void match(Settings::Side side) {
  begin(side);

  // FIRST STACK: the preload
  // Straight out along the row so turning doesn't clip the match load
  pointTo(120, -150, 1500, CHAIN_DRIVE);
  if (!finished()) return;

  // Curve to the goal, ending facing it (heading 0), lift up to carry height
  moveTo(120, -75, 0, 3000);
  Components::liftTo(25.5);
  // Start lowering onto the goal once within 5 cm, while the drive settles
  if (!waitUntilNear(120, -75, 5)) return;
  Components::liftTo(18);
  if (!finished() || !liftNear(18, 2)) return;

  // Let go and back straight out to the row, without waiting for the claw
  Components::clawOpen();
  straightToY(-150, 3000, CHAIN_BACK);
  if (!finished()) return;

  // SECOND STACK: from the match load
  // Turn to our wall and drive up to the load, lift down to intake height
  Components::liftTo(LIFT_BOTTOM);
  turnTo(90, 1500, CHAIN_TURN);
  if (!finished()) return;
  pointTo(160, -150, 2000, CHAIN_DRIVE);
  if (!finished() || !liftNear(LIFT_BOTTOM, 2)) return;

  // Intake for 1.5 s while pushing gently so it stays against the wall
  Components::clawIntake(true);
  if (!pushForward(25, 1500)) return;

  // Back out along the row. The intake keeps pulling for 1.5 s on its own,
  // and the lift rises 5 degrees 0.5 s in, once it's clear of the load
  pointTo(120, -150, 2000, CHAIN_BACK);
  after(1500, [] { Components::clawIntake(false); });
  if (!wait(500)) return;
  Components::liftTo(LIFT_BOTTOM + 5);
  if (!finished()) return;

  // Turn to the goal and drive in, lift up to carry height (GUESS)
  turnTo(0, 1500, CHAIN_TURN);
  if (!finished()) return;
  pointTo(120, -75, 3000);
  Components::liftTo(40);
  // Lower onto the first stack (GUESS) once within 5 cm
  if (!waitUntilNear(120, -75, 5)) return;
  Components::liftTo(32.5);
  if (!finished() || !liftNear(32.5, 2)) return;

  // Let go and back straight out
  Components::clawOpen();
  straightToY(-150, 3000, CHAIN_BACK);
  finished();
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
