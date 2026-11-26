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

// REACH: a drive with a minimum speed. It never fades out and stops short of
// the point, it keeps going until it crosses the point's line, then stops.
// Higher is quicker but slides farther past
constexpr lemlib::MoveToPointParams REACH{.minSpeed = 20};
constexpr lemlib::MoveToPointParams REACH_BACK{.forwards = false,
                                               .minSpeed = 20};

// The lift's lowest angle, also the intake height
constexpr double LIFT_BOTTOM = LiftConfig::MIN_ANGLE;

} // namespace

// Straight lines and full 90s: every turn finishes before the next drive, so
// the robot stays square instead of curving into its lines
void match(Settings::Side side) {
  begin(side);

  // FIRST STACK: the preload
  // Straight out along the row so turning doesn't clip the match load
  pointTo(120, -150, 1500, REACH);
  if (!finished()) return;

  // Turn to the goal (heading 0), lifting to carry height
  Components::liftTo(25.5);
  turnTo(0, 1500);
  if (!finished()) return;

  // Straight in. Start lowering onto the goal once within 5 cm
  pointTo(120, -68, 2500, REACH);
  if (!waitUntilNear(120, -68, 5)) return;
  Components::liftTo(18);
  if (!finished() || !liftNear(18, 2)) return;

  // Let go and back straight out to the row, without waiting for the claw
  Components::clawOpen();
  straightToY(-150, 2500, REACH_BACK);
  if (!finished()) return;

  // SECOND STACK: from the match load
  // Turn to our wall and drive up to the load, lift down to intake height
  Components::liftTo(LIFT_BOTTOM);
  turnTo(90, 1500);
  if (!finished()) return;
  // TODO: -150 is the start row, beside the load. Drive to the load by hand
  // and put its Y here (and in the drive back out below)
  pointTo(160, -150, 2000, REACH);
  if (!finished() || !liftNear(LIFT_BOTTOM, 2)) return;

  // Intake for 1.5 s while pushing gently so it stays against the wall
  Components::clawIntake(true);
  if (!pushForward(25, 1500)) return;

  // Back out along the row. The intake keeps pulling for 1.5 s on its own,
  // and the lift rises 5 degrees 0.5 s in, once it's clear of the load
  pointTo(120, -150, 2000, REACH_BACK);
  after(1500, [] { Components::clawIntake(false); });
  if (!wait(500)) return;
  Components::liftTo(LIFT_BOTTOM + 5);
  if (!finished()) return;

  // Turn to the goal, lifting to carry height (GUESS)
  turnTo(0, 1500);
  Components::liftTo(40);
  if (!finished()) return;

  // Straight in. Lower onto the first stack (GUESS) once within 5 cm
  pointTo(120, -68, 2500, REACH);
  if (!waitUntilNear(120, -68, 5)) return;
  Components::liftTo(32.5);
  if (!finished() || !liftNear(32.5, 2)) return;

  // Let go and back straight out to the row, lift down for the next load
  Components::clawOpen();
  straightToY(-150, 2500, REACH_BACK);
  Components::liftTo(LIFT_BOTTOM);
  if (!finished()) return;

  // Back to the match load, the same square way as the first trip
  turnTo(90, 1500);
  if (!finished()) return;
  pointTo(160, -150, 2000, REACH);
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
