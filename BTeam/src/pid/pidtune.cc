// PID TUNER
// Edits the real PIDs in memory, runs test moves, and measures each one.
// Values reset when the program restarts, so copy good ones into pidconfig.hpp
//
// Y: swap between the graph and the tune page
// Graph page: A runs the test, A or B or a stick stops it
// Tune page: arrows move, A is enter, B goes back
#include "pidtune.hpp"
#include "../components/components.hpp"
#include "../components/liftcore.hpp"
#include "../hardware/hardware.hpp"
#include "../interface/ctrlscreen.hpp"
#include "../macro/macro.hpp"
#include "liftpid.hpp"
#include "pidconfig.hpp"
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
constexpr float TURN_TESTS[] = {15, 45, 90, 180};  // degrees
constexpr float LIFT_TESTS[] = {15, 40, 80, 120};  // degrees at the lift
constexpr int TEST_COUNT = 4;
constexpr int MOVE_COUNT = TEST_COUNT * 2;
// Keeps measuring this long after a move ends, to catch bounce
constexpr std::uint32_t SETTLE_PAUSE = 350;
// Lift tests stay this many degrees away from the limits
constexpr float LIFT_ROOM_MARGIN = 3;
// Longest wait for the drive to stop before changing its PID, in milliseconds
constexpr std::uint32_t DRIVE_STOP_WAIT = 200;

struct TestSettings {
  const float *sizes;
  int timeout;          // longest each move can take, in milliseconds
  float tolerance;      // counts as settled once it stays this close
  float overshootLimit; // when the hint suggests a change
  float slowTime;       // seconds
};

constexpr TestSettings TESTS[SYSTEM_COUNT] = {
    {DRIVE_TESTS, 3000, 1, 1, 2.0f}, // inches
    {TURN_TESTS, 2000, 1, 2, 1.3f},  // degrees
    {LIFT_TESTS, 2500, 2, 3, 1.5f},  // degrees
};

// CONTROLS
constexpr std::uint32_t REPEAT_DELAY = 350; // before a held arrow repeats
constexpr std::uint32_t REPEAT_RATE = 90;   // between repeats
constexpr int STOP_STICK = 25;              // moving a stick stops a test

// Digits are 100s, 10s, 1s, tenths, hundredths, stored in hundredths
constexpr int DIGIT_PLACES[DIGIT_COUNT] = {10000, 1000, 100, 10, 1};
constexpr int MAX_GAIN = 99999;
constexpr int START_DIGIT = 2;

enum class RunStep { Idle, Moving, Settling };

struct RepeatTimer {
  std::uint32_t next = 0;
};

bool gainsLoaded = false;
RepeatTimer upRepeat, downRepeat, leftRepeat, rightRepeat;

// Tests zero the odometry, so the field position comes back on exit
bool poseSaved = false;
lemlib::Pose savedPose(0, 0, 0);

RunStep runStep = RunStep::Idle;
int runSystem = SYS_DRIVE;
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
float liftBase = 0; // height the lift test starts and returns to
float liftSign = 1; // the lift test moves toward whichever limit is farther
float liftRoom = 0;

int toHundredths(float value) {
  return std::clamp(static_cast<int>(std::lround(value * 100)), 0, MAX_GAIN);
}

float wrap180(float degrees) {
  return degrees - 360.0f * std::round(degrees / 360.0f);
}

// NaN if the rotation sensor is unplugged
float readLift() {
  const std::int32_t angle = Hardware::liftRotation.get_angle();
  return angle == PROS_ERR
             ? NAN
             : static_cast<float>(LiftCore::angleToHeight(angle / 100.0));
}

float readPosition() {
  if (runSystem == SYS_LIFT) {
    return readLift();
  }
  const lemlib::Pose pose = chassis.getPose();
  return runSystem == SYS_TURN ? pose.theta : pose.y;
}

// How far the robot has to go from here, measured in this move's direction
float neededTravel(float current, float target) {
  const float difference = (target - current) * moveDirection;
  if (runSystem != SYS_TURN) {
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

void setGains(int system, float kP, float kI, float kD) {
  View &v = view();
  v.gains[system][0] = toHundredths(kP);
  v.gains[system][1] = toHundredths(kI);
  v.gains[system][2] = toHundredths(kD);
}

// Loads the values written in pidconfig.hpp
void loadCodeGains(int system) {
  if (system == SYS_DRIVE) {
    setGains(SYS_DRIVE, PidConfig::DRIVE.kP, PidConfig::DRIVE.kI,
             PidConfig::DRIVE.kD);
  } else if (system == SYS_TURN) {
    setGains(SYS_TURN, PidConfig::TURN.kP, PidConfig::TURN.kI,
             PidConfig::TURN.kD);
  } else {
    setGains(SYS_LIFT, PidConfig::Lift::KP, PidConfig::Lift::KI,
             PidConfig::Lift::KD);
  }
}

float gainValue(int system, int term) {
  return view().gains[system][term].load() / 100.0f;
}

// PID gains are const, so the object is rebuilt with the new values
void setPid(lemlib::PID &pid, int system, float windupRange) {
  const float kP = gainValue(system, 0);
  const float kI = gainValue(system, 1);
  const float kD = gainValue(system, 2);
  pid.~PID();
  ::new (static_cast<void *>(&pid)) lemlib::PID(kP, kI, kD, windupRange, true);
}

void stopDrive() {
  chassis.cancelAllMotions();
  chassis.tank(0, 0, true);
}

void applyGains() {
  // A move still ending could be using the PID objects
  const std::uint32_t start = pros::millis();
  while (chassis.isInMotion() && pros::millis() - start < DRIVE_STOP_WAIT) {
    pros::delay(5);
  }
  setPid(chassis.lateralPID, SYS_DRIVE, PidConfig::DRIVE.windup);
  setPid(chassis.angularPID, SYS_TURN, PidConfig::TURN.windup);
  LiftPid::setGains({gainValue(SYS_LIFT, 0), gainValue(SYS_LIFT, 1),
                     gainValue(SYS_LIFT, 2)});
}

void printGains() {
  for (int system = 0; system < SYSTEM_COUNT; system++) {
    std::printf("%s kP %.2f kI %.2f kD %.2f\n", SYSTEM_NAMES[system],
                gainValue(system, 0), gainValue(system, 1),
                gainValue(system, 2));
  }
}

void startMove() {
  View &v = view();
  const TestSettings &test = TESTS[runSystem];
  const bool lift = runSystem == SYS_LIFT;
  const bool outward = moveIndex % 2 == 0;

  moveSize = test.sizes[moveIndex / 2];
  if (lift) {
    moveSize = std::min(moveSize, liftRoom);
  }
  moveDirection = (outward ? 1.0f : -1.0f) * (lift ? liftSign : 1.0f);

  float target = outward ? moveSize : 0.0f;
  if (lift) {
    target = outward ? liftBase + liftSign * moveSize : liftBase;
  }

  lastRaw = readPosition();
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

  if (lift) {
    Components::liftBy(target - lastRaw);
  } else if (runSystem == SYS_TURN) {
    chassis.turnToHeading(
        target, test.timeout,
        {.direction = outward
                          ? lemlib::AngularDirection::CW_CLOCKWISE
                          : lemlib::AngularDirection::CCW_COUNTERCLOCKWISE});
  } else {
    chassis.moveToPoint(0, target, test.timeout, {.forwards = outward});
  }
}

void startRun() {
  View &v = view();
  const int system = v.system.load();

  if (system == SYS_LIFT) {
    const float height = readLift();
    if (std::isnan(height)) {
      v.phase = PHASE_STOPPED;
      v.hint = HINT_NO_SENSOR;
      return;
    }
    const float up = static_cast<float>(LiftCore::range()) - height;
    liftSign = up >= height ? 1.0f : -1.0f;
    liftBase = height;
    liftRoom = std::max(up, height) - LIFT_ROOM_MARGIN;
  }

  stopDrive();
  applyGains();
  runSystem = system;
  if (system != SYS_LIFT) {
    if (!poseSaved) {
      savedPose = chassis.getPose();
      poseSaved = true;
    }
    chassis.setPose(0, 0, 0);
  }

  moveIndex = 0;
  v.testSystem = system;
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
  if (runSystem == SYS_LIFT) {
    Components::liftBy(0); // hold wherever the lift is now
  }
  runStep = RunStep::Idle;
  view().phase = phase;
}

void sampleMove() {
  View &v = view();
  const float raw = readPosition();
  if (std::isnan(raw)) {
    v.hint = HINT_NO_SENSOR;
    stopRun(PHASE_STOPPED);
    return;
  }
  const float change =
      runSystem == SYS_TURN ? wrap180(raw - lastRaw) : raw - lastRaw;
  lastRaw = raw;
  traveled += change;

  const float progress = traveled * moveDirection;
  const float error = moveNeeded - progress;
  moveOvershoot = std::max(moveOvershoot, -error);
  moveError = std::fabs(error);
  if (moveError > TESTS[runSystem].tolerance) {
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
  const TestSettings &test = TESTS[runSystem];
  if (v.worstOvershoot.load() > test.overshootLimit) {
    return HINT_OVERSHOOT;
  }
  if (v.worstError.load() > test.tolerance) {
    return HINT_SHORT;
  }
  if (v.worstSettle.load() > test.slowTime) {
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

bool moveEnded() {
  if (runSystem == SYS_LIFT) {
    return Components::liftSettled() ||
           pros::millis() - moveStart >=
               static_cast<std::uint32_t>(TESTS[SYS_LIFT].timeout);
  }
  return !chassis.isInMotion();
}

void updateRun(const Controls &driver) {
  const bool stickMoved =
      std::abs(Hardware::controller.get_analog(driver.leftY)) > STOP_STICK ||
      std::abs(Hardware::controller.get_analog(driver.rightX)) > STOP_STICK;
  if (driver.menuSelect.newPress() || driver.menuBack.newPress() ||
      stickMoved) {
    stopRun(PHASE_STOPPED);
    return;
  }

  sampleMove();
  if (runStep == RunStep::Moving) {
    if (moveEnded()) {
      runStep = RunStep::Settling;
      settleStart = pros::millis();
    }
  } else if (runStep == RunStep::Settling &&
             pros::millis() - settleStart >= SETTLE_PAUSE) {
    finishMove();
  }
}

// Each digit rolls between 0 and 9 without touching the digits beside it
void changeDigit(int direction) {
  View &v = view();
  const int system = v.system.load();
  const int term = v.valueRow.load();
  const int place = DIGIT_PLACES[v.digit.load()];
  const int value = v.gains[system][term].load();
  const int digit = (value / place) % 10;
  const int next = (digit + direction + 10) % 10;
  v.gains[system][term] = value + (next - digit) * place;
}

void updateDigit(const Controls &driver) {
  View &v = view();
  if (driver.menuSelect.newPress() || driver.menuBack.newPress()) {
    v.focus = FOCUS_VALUES;
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

void updateValues(const Controls &driver) {
  View &v = view();
  const int row = v.valueRow.load();
  if (driver.menuBack.newPress()) {
    v.focus = FOCUS_TEST;
    return;
  }
  if (driver.menuSelect.newPress()) {
    if (row == RESET_ROW) {
      loadCodeGains(v.system.load());
    } else {
      v.digit = START_DIGIT;
      v.focus = FOCUS_DIGIT;
    }
    return;
  }
  if (pressedOrRepeat(driver.menuUp, upRepeat)) {
    if (row == 0) {
      v.focus = FOCUS_TEST;
    } else {
      v.valueRow = row - 1;
    }
  }
  if (pressedOrRepeat(driver.menuDown, downRepeat)) {
    v.valueRow = std::min(row + 1, RESET_ROW);
  }
}

void updateDropdown(const Controls &driver) {
  View &v = view();
  if (driver.menuBack.newPress()) {
    v.focus = FOCUS_TEST;
    return;
  }
  if (driver.menuSelect.newPress()) {
    v.system = v.dropdownRow.load();
    v.valueRow = 0;
    v.focus = FOCUS_VALUES;
    return;
  }
  const int row = v.dropdownRow.load();
  if (pressedOrRepeat(driver.menuUp, upRepeat)) {
    v.dropdownRow = (row - 1 + SYSTEM_COUNT) % SYSTEM_COUNT;
  }
  if (pressedOrRepeat(driver.menuDown, downRepeat)) {
    v.dropdownRow = (row + 1) % SYSTEM_COUNT;
  }
}

void updateTestRow(const Controls &driver) {
  View &v = view();
  if (driver.menuSelect.newPress()) {
    v.dropdownRow = v.system.load();
    v.focus = FOCUS_DROPDOWN;
    return;
  }
  const int system = v.system.load();
  if (pressedOrRepeat(driver.menuLeft, leftRepeat)) {
    v.system = (system - 1 + SYSTEM_COUNT) % SYSTEM_COUNT;
  }
  if (pressedOrRepeat(driver.menuRight, rightRepeat)) {
    v.system = (system + 1) % SYSTEM_COUNT;
  }
  if (driver.menuDown.newPress()) {
    v.valueRow = 0;
    v.focus = FOCUS_VALUES;
  }
}

void updateTunePage(const Controls &driver) {
  switch (view().focus.load()) {
  case FOCUS_TEST:
    updateTestRow(driver);
    break;
  case FOCUS_DROPDOWN:
    updateDropdown(driver);
    break;
  case FOCUS_VALUES:
    updateValues(driver);
    break;
  default:
    updateDigit(driver);
    break;
  }
}

const char *shortHint(int hint) {
  switch (hint) {
  case HINT_OVERSHOOT:
    return "Raise kD/low kP";
  case HINT_SHORT:
  case HINT_SLOW:
    return "Raise kP";
  case HINT_GOOD:
    return "Looks good";
  case HINT_NO_SENSOR:
    return "No lift sensor";
  default:
    return "A: run again";
  }
}

void drawGraphLines() {
  View &v = view();
  char line[32];
  switch (v.phase.load()) {
  case PHASE_RUNNING:
    std::snprintf(line, sizeof(line), "Move %d/%d", v.moveNumber.load(),
                  v.moveCount.load());
    CtrlScreen::write(1, line);
    CtrlScreen::write(2, "A or B: stop");
    break;
  case PHASE_DONE:
    std::snprintf(line, sizeof(line), "OS%.1f ER%.1f",
                  static_cast<double>(v.worstOvershoot.load()),
                  static_cast<double>(v.worstError.load()));
    CtrlScreen::write(1, line);
    CtrlScreen::write(2, shortHint(v.hint.load()));
    break;
  case PHASE_STOPPED:
    CtrlScreen::write(1, "Stopped");
    CtrlScreen::write(2, shortHint(v.hint.load()));
    break;
  default:
    CtrlScreen::write(1, "A: run test");
    CtrlScreen::write(2, "Y: edit gains");
    break;
  }
}

void drawTuneLines() {
  View &v = view();
  const int system = v.system.load();
  const int focus = v.focus.load();
  const int row = v.valueRow.load();
  char line[32];

  if (focus == FOCUS_TEST) {
    std::snprintf(line, sizeof(line), "Test: <%s>", SYSTEM_NAMES[system]);
    CtrlScreen::write(1, line);
    CtrlScreen::write(2, "A:list v:values");
  } else if (focus == FOCUS_DROPDOWN) {
    std::snprintf(line, sizeof(line), "> %s",
                  SYSTEM_NAMES[v.dropdownRow.load()]);
    CtrlScreen::write(1, line);
    CtrlScreen::write(2, "A: select");
  } else if (row == RESET_ROW) {
    CtrlScreen::write(1, "Reset to code");
    CtrlScreen::write(2, "A: reset");
  } else {
    const int value = v.gains[system][row].load();
    std::snprintf(line, sizeof(line), "%s %03d.%02d", TERM_NAMES[row],
                  value / 100, value % 100);
    CtrlScreen::write(1, line);
    if (focus == FOCUS_DIGIT) {
      // Points at the digit being edited, past the name and decimal point
      const int digit = v.digit.load();
      const int column = 3 + digit + (digit >= 3 ? 1 : 0);
      std::snprintf(line, sizeof(line), "%*s^", column, "");
      CtrlScreen::write(2, line);
    } else {
      CtrlScreen::write(2, "A edit  Y graph");
    }
  }
}

void drawController() {
  View &v = view();
  const bool graphPage = v.page.load() == PAGE_GRAPH;
  char line[32];
  std::snprintf(line, sizeof(line), "PID %s %s", SYSTEM_NAMES[v.system.load()],
                graphPage ? "GRAPH" : "TUNE");
  CtrlScreen::write(0, line);
  if (graphPage) {
    drawGraphLines();
  } else {
    drawTuneLines();
  }
}

} // namespace

void start() { startScreen(); }

void open() {
  View &v = view();
  Macro::cancel();
  Components::lift(0);
  stopDrive();
  if (!gainsLoaded) {
    for (int system = 0; system < SYSTEM_COUNT; system++) {
      loadCodeGains(system);
    }
    gainsLoaded = true;
  }
  v.page = PAGE_GRAPH;
  v.focus = FOCUS_TEST;
  v.phase = PHASE_READY;
  v.hint = HINT_NONE;
  v.open = true;
}

void close() {
  View &v = view();
  if (runStep != RunStep::Idle) {
    stopRun(PHASE_STOPPED);
  }
  // Tuned values stay active, so auton and the lift use them until a restart
  applyGains();
  printGains();
  if (poseSaved) {
    chassis.setPose(savedPose);
    poseSaved = false;
  }
  v.open = false;
}

void update(const Controls &driver) {
  View &v = view();
  if (driver.pidMode.newPress()) {
    v.page = v.page.load() == PAGE_GRAPH ? PAGE_TUNE : PAGE_GRAPH;
  }

  if (runStep != RunStep::Idle) {
    updateRun(driver);
  } else if (v.page.load() == PAGE_GRAPH) {
    if (driver.menuSelect.newPress()) {
      startRun();
    }
  } else {
    updateTunePage(driver);
  }
  drawController();
}

} // namespace PidTune
