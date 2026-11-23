// PID TUNER
// Live edits the real LemLib PIDs, runs test moves, and measures each one.
// Values reset when the program restarts, so copy good ones into drivetrain.cc
//
// Up/Down: pick a row, or change a digit while editing
// Left/Right: pick a digit while editing
// L1: enter, B: leave a digit or stop a test
#include "pidtune.hpp"
#include "../components/components.hpp"
#include "../hardware/hardware.hpp"
#include "../lemlib/drivetrain.hpp"
#include "../macro/macro.hpp"
#include "pidscreen.hpp"
#include "pidstate.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <new>

namespace PidTune {

View &view() {
  static View state;
  return state;
}

namespace {

// TEST MOVES, each one goes out and comes back
constexpr float DRIVE_TESTS[] = {6, 12, 24, 36};   // inches
constexpr float STEER_TESTS[] = {15, 45, 90, 180}; // degrees
constexpr int TEST_COUNT = 4;
constexpr int MOVE_COUNT = TEST_COUNT * 2;
// Longest each move can take, in milliseconds
constexpr int DRIVE_TIMEOUT = 3000;
constexpr int STEER_TIMEOUT = 2000;
// Keeps measuring this long after a move ends, to catch bounce
constexpr std::uint32_t SETTLE_PAUSE = 350;

// MEASURING
// Counts as settled once it stays this close to the target
constexpr float DRIVE_TOLERANCE = 1; // inches
constexpr float STEER_TOLERANCE = 1; // degrees
// When the hint suggests a change
constexpr float DRIVE_OVERSHOOT_LIMIT = 1; // inches
constexpr float STEER_OVERSHOOT_LIMIT = 2; // degrees
constexpr float DRIVE_SLOW_TIME = 2.0f;    // seconds
constexpr float STEER_SLOW_TIME = 1.3f;    // seconds

// CONTROLS
constexpr std::uint32_t OPEN_HOLD_TIME = 1000;
constexpr std::uint32_t REPEAT_DELAY = 350; // before a held arrow repeats
constexpr std::uint32_t REPEAT_RATE = 90;   // between repeats
constexpr int STOP_STICK = 25;              // moving a stick stops a test

// Digits are 100s, 10s, 1s, tenths, hundredths, stored in hundredths
constexpr int DIGIT_PLACES[DIGIT_COUNT] = {10000, 1000, 100, 10, 1};
constexpr int MAX_GAIN = 99999;
constexpr int START_DIGIT = 2;

enum class RunStep { Idle, Moving, Settling };

// LemLib keeps these protected, this reads the values from drivetrain.cc
struct ChassisSettings : lemlib::Chassis {
  static const lemlib::ControllerSettings &lateral(const lemlib::Chassis &c) {
    return c.*(&ChassisSettings::lateralSettings);
  }
  static const lemlib::ControllerSettings &angular(const lemlib::Chassis &c) {
    return c.*(&ChassisSettings::angularSettings);
  }
};

struct RepeatTimer {
  std::uint32_t next = 0;
};

bool holdingOpen = false;
std::uint32_t holdStart = 0;
bool gainsLoaded = false;
RepeatTimer upRepeat, downRepeat, leftRepeat, rightRepeat;

RunStep runStep = RunStep::Idle;
bool steerRun = false;
int moveIndex = 0;
float moveSize = 0;
float moveNeeded = 0; // real distance left, since the last move can miss a bit
float moveDirection = 1;
float lastRaw = 0;
float traveled = 0;
float moveOvershoot = 0;
float moveSettle = 0;
float moveError = 0;
std::uint32_t moveStart = 0;
std::uint32_t settleStart = 0;

int toHundredths(float value) {
  return std::clamp(static_cast<int>(std::lround(value * 100)), 0, MAX_GAIN);
}

float wrap180(float degrees) {
  return degrees - 360.0f * std::round(degrees / 360.0f);
}

// How far the robot has to go from here, measured in this move's direction
float neededTravel(float current, float target) {
  const float difference = (target - current) * moveDirection;
  if (!steerRun) {
    return difference;
  }
  float needed = std::fmod(difference, 360.0f);
  if (needed < -90) {
    needed += 360;
  } else if (needed > 270) {
    needed -= 360;
  }
  return needed;
}

bool pressedOrRepeat(const Button &button, RepeatTimer &timer) {
  const std::uint32_t now = pros::millis();
  if (button.newPress()) {
    timer.next = now + REPEAT_DELAY;
    return true;
  }
  if (button.pressed() && now >= timer.next) {
    timer.next = now + REPEAT_RATE;
    return true;
  }
  return false;
}

void loadCodeGains() {
  View &v = view();
  const lemlib::ControllerSettings &drive = ChassisSettings::lateral(chassis);
  const lemlib::ControllerSettings &steer = ChassisSettings::angular(chassis);
  v.gains[DRIVE_KP] = toHundredths(drive.kP);
  v.gains[DRIVE_KI] = toHundredths(drive.kI);
  v.gains[DRIVE_KD] = toHundredths(drive.kD);
  v.gains[STEER_KP] = toHundredths(steer.kP);
  v.gains[STEER_KI] = toHundredths(steer.kI);
  v.gains[STEER_KD] = toHundredths(steer.kD);
}

// PID gains are const, so the object is rebuilt with the new values
void setPid(lemlib::PID &pid, int firstGain, float windupRange) {
  View &v = view();
  const float kP = v.gains[firstGain].load() / 100.0f;
  const float kI = v.gains[firstGain + 1].load() / 100.0f;
  const float kD = v.gains[firstGain + 2].load() / 100.0f;
  pid.~PID();
  ::new (static_cast<void *>(&pid)) lemlib::PID(kP, kI, kD, windupRange, true);
}

void applyGains() {
  setPid(chassis.lateralPID, DRIVE_KP,
         ChassisSettings::lateral(chassis).windupRange);
  setPid(chassis.angularPID, STEER_KP,
         ChassisSettings::angular(chassis).windupRange);
}

void printGains() {
  View &v = view();
  const auto value = [&v](int gain) { return v.gains[gain].load() / 100.0; };
  std::printf("DRIVE kP %.2f kI %.2f kD %.2f | STEER kP %.2f kI %.2f kD %.2f\n",
              value(DRIVE_KP), value(DRIVE_KI), value(DRIVE_KD),
              value(STEER_KP), value(STEER_KI), value(STEER_KD));
}

void stopDrive() {
  chassis.cancelAllMotions();
  chassis.tank(0, 0, true);
}

void startMove() {
  View &v = view();
  const float *tests = steerRun ? STEER_TESTS : DRIVE_TESTS;
  const bool outward = moveIndex % 2 == 0;
  moveSize = tests[moveIndex / 2];
  moveDirection = outward ? 1.0f : -1.0f;
  const float target = outward ? moveSize : 0.0f;

  const lemlib::Pose pose = chassis.getPose();
  lastRaw = steerRun ? pose.theta : pose.y;
  moveNeeded = neededTravel(lastRaw, target);
  traveled = 0;
  moveOvershoot = 0;
  moveSettle = 0;
  moveError = std::fabs(moveNeeded);
  moveStart = pros::millis();

  v.moveNumber = moveIndex + 1;
  v.moveSize = moveSize;
  v.graphTarget = moveNeeded;
  v.graphCount = 0;
  v.graphEpoch.fetch_add(1);
  runStep = RunStep::Moving;

  if (steerRun) {
    chassis.turnToHeading(
        target, STEER_TIMEOUT,
        {.direction = outward
                          ? lemlib::AngularDirection::CW_CLOCKWISE
                          : lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
  } else {
    chassis.moveToPoint(0, target, DRIVE_TIMEOUT, {.forwards = outward});
  }
}

void startRun(bool steer) {
  View &v = view();
  applyGains();
  stopDrive();
  chassis.setPose(0, 0, 0);

  steerRun = steer;
  moveIndex = 0;
  v.steerTest = steer;
  v.moveCount = MOVE_COUNT;
  v.phase = PHASE_RUNNING;
  v.hint = HINT_NONE;
  v.overshoot = 0;
  v.finalError = 0;
  v.settleTime = 0;
  v.worstOvershoot = 0;
  v.worstError = 0;
  v.worstSettle = 0;
  startMove();
}

void stopRun(Phase phase) {
  stopDrive();
  runStep = RunStep::Idle;
  view().phase = phase;
}

void sampleMove() {
  View &v = view();
  const lemlib::Pose pose = chassis.getPose();
  const float raw = steerRun ? pose.theta : pose.y;
  const float change = steerRun ? wrap180(raw - lastRaw) : raw - lastRaw;
  lastRaw = raw;
  traveled += change;

  const float progress = traveled * moveDirection;
  const float error = moveNeeded - progress;
  const float tolerance = steerRun ? STEER_TOLERANCE : DRIVE_TOLERANCE;
  moveOvershoot = std::max(moveOvershoot, -error);
  moveError = std::fabs(error);
  if (moveError > tolerance) {
    moveSettle = (pros::millis() - moveStart) / 1000.0f;
  }

  const int count = v.graphCount.load();
  if (count < GRAPH_POINTS) {
    v.graph[count] = progress;
    v.graphCount = count + 1;
  }
  v.overshoot = moveOvershoot;
  v.finalError = moveError;
  v.settleTime = moveSettle;
}

Hint chooseHint() {
  View &v = view();
  const float overshootLimit =
      steerRun ? STEER_OVERSHOOT_LIMIT : DRIVE_OVERSHOOT_LIMIT;
  const float errorLimit = steerRun ? STEER_TOLERANCE : DRIVE_TOLERANCE;
  const float slowTime = steerRun ? STEER_SLOW_TIME : DRIVE_SLOW_TIME;
  if (v.worstOvershoot.load() > overshootLimit) {
    return HINT_OVERSHOOT;
  }
  if (v.worstError.load() > errorLimit) {
    return HINT_SHORT;
  }
  if (v.worstSettle.load() > slowTime) {
    return HINT_SLOW;
  }
  return HINT_GOOD;
}

void finishMove() {
  View &v = view();
  v.worstOvershoot = std::max(v.worstOvershoot.load(), moveOvershoot);
  v.worstError = std::max(v.worstError.load(), moveError);
  v.worstSettle = std::max(v.worstSettle.load(), moveSettle);

  moveIndex++;
  if (moveIndex < MOVE_COUNT) {
    startMove();
    return;
  }
  v.hint = chooseHint();
  stopRun(PHASE_DONE);
  printGains();
}

void updateRun(const Controls &driver) {
  const bool stickMoved =
      std::abs(Hardware::controller.get_analog(driver.leftY)) > STOP_STICK ||
      std::abs(Hardware::controller.get_analog(driver.rightX)) > STOP_STICK;
  if (driver.tuneBack.newPress() || stickMoved) {
    stopRun(PHASE_STOPPED);
    return;
  }

  sampleMove();
  if (runStep == RunStep::Moving) {
    if (!chassis.isInMotion()) {
      runStep = RunStep::Settling;
      settleStart = pros::millis();
    }
  } else if (pros::millis() - settleStart >= SETTLE_PAUSE) {
    finishMove();
  }
}

void openTuner() {
  View &v = view();
  Macro::cancel();
  Components::lift(0);
  stopDrive();
  if (!gainsLoaded) {
    loadCodeGains();
    gainsLoaded = true;
  }
  v.row = DRIVE_KP;
  v.digit = START_DIGIT;
  v.editing = false;
  v.phase = PHASE_READY;
  v.hint = HINT_NONE;
  v.open = true;
}

void closeTuner() {
  View &v = view();
  applyGains();
  printGains();
  v.editing = false;
  v.open = false;
}

void changeDigit(int direction) {
  View &v = view();
  const int row = v.row.load();
  const int place = DIGIT_PLACES[v.digit.load()];
  v.gains[row] =
      std::clamp(v.gains[row].load() + direction * place, 0, MAX_GAIN);
}

void updateEditing(const Controls &driver) {
  View &v = view();
  if (driver.menuSelect.newPress() || driver.tuneBack.newPress()) {
    v.editing = false;
    return;
  }
  if (pressedOrRepeat(driver.menuUp, upRepeat)) {
    changeDigit(1);
  }
  if (pressedOrRepeat(driver.menuDown, downRepeat)) {
    changeDigit(-1);
  }
  if (pressedOrRepeat(driver.menuLeft, leftRepeat)) {
    v.digit = std::max(v.digit.load() - 1, 0);
  }
  if (pressedOrRepeat(driver.menuRight, rightRepeat)) {
    v.digit = std::min(v.digit.load() + 1, DIGIT_COUNT - 1);
  }
}

void moveRow(int change) {
  View &v = view();
  v.row = (v.row.load() + change + ROW_COUNT) % ROW_COUNT;
}

void activateRow() {
  View &v = view();
  const int row = v.row.load();
  if (row < GAIN_COUNT) {
    v.digit = START_DIGIT;
    v.editing = true;
  } else if (row == RUN_DRIVE) {
    startRun(false);
  } else if (row == RUN_STEER) {
    startRun(true);
  } else if (row == RESET_GAINS) {
    loadCodeGains();
    applyGains();
    v.phase = PHASE_READY;
    v.hint = HINT_NONE;
  } else {
    closeTuner();
  }
}

void updateMenu(const Controls &driver) {
  View &v = view();
  if (driver.menuSelect.newPress()) {
    activateRow();
    return;
  }
  const bool onActions = v.row.load() >= GAIN_COUNT;
  if (pressedOrRepeat(driver.menuUp, upRepeat)) {
    moveRow(-1);
  }
  if (pressedOrRepeat(driver.menuDown, downRepeat)) {
    moveRow(1);
  }
  if (onActions && pressedOrRepeat(driver.menuLeft, leftRepeat)) {
    moveRow(-1);
  }
  if (onActions && pressedOrRepeat(driver.menuRight, rightRepeat)) {
    moveRow(1);
  }
}

} // namespace

void start() { startScreen(); }

bool update(const Controls &driver) {
  View &v = view();
  if (!v.open.load()) {
    if (!driver.tuneOpenA.pressed() || !driver.tuneOpenB.pressed()) {
      holdingOpen = false;
      return false;
    }
    const std::uint32_t now = pros::millis();
    if (!holdingOpen) {
      holdingOpen = true;
      holdStart = now;
    }
    if (now - holdStart < OPEN_HOLD_TIME) {
      return false;
    }
    holdingOpen = false;
    openTuner();
    return true;
  }

  if (runStep != RunStep::Idle) {
    updateRun(driver);
  } else if (v.editing.load()) {
    updateEditing(driver);
  } else {
    updateMenu(driver);
  }
  return true;
}

} // namespace PidTune
