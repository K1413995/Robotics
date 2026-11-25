// TEST MODES
// A double tap of A only enters auto mode from driver mode, because A is the
// enter button in the PID tuner and a fast double enter shouldn't leave it
#include "testmode.hpp"
#include "../auton/autonrun.hpp"
#include "../components/components.hpp"
#include "../controller/controller.hpp"
#include "../macro/macro.hpp"
#include "../pid/pidtune.hpp"
#include "ctrlscreen.hpp"
#include "interface.hpp"
#include "pros/rtos.hpp"
#include "settings.hpp"

#include <cstdint>
#include <cstdio>

namespace TestMode {
namespace {

// Longest gap between the two presses of a double tap, in milliseconds
constexpr std::uint32_t DOUBLE_TAP_MS = 350;

enum class Mode { Driver, Auto, Pid };
enum class AutoState { Ready, Running, Done, Stopped };

class DoubleTap {
  std::uint32_t lastPress = 0;
  bool armed = false;

public:
  bool update(const Button &button) {
    if (!button.newPress()) {
      return false;
    }
    const std::uint32_t now = pros::millis();
    if (armed && now - lastPress <= DOUBLE_TAP_MS) {
      armed = false;
      return true;
    }
    armed = true;
    lastPress = now;
    return false;
  }
};

Mode mode = Mode::Driver;
Mode nextMode = Mode::Driver;
bool stopping = false; // waiting for a canceled auton to exit
DoubleTap autoTaps, driverTaps, pidTaps;

AutoState autoState = AutoState::Ready;
bool wallReadFailed = false; // the next A runs anyway
// An extra fast tap of A can't start the robot before this time
std::uint32_t startAllowedAt = 0;

const char *modeLabel(Mode which) {
  switch (which) {
  case Mode::Auto:
    return "AUTO";
  case Mode::Pid:
    return "PID";
  case Mode::Driver:
    break;
  }
  return "DRIVER";
}

const char *sideName() {
  return Settings::side() == Settings::Side::Left ? "LEFT" : "RIGHT";
}

void haltRobot() {
  Macro::cancel();
  chassis.cancelAllMotions();
  chassis.tank(0, 0, true);
  Components::clawIntake(false); // a canceled auton could leave it on
  Components::lift(0);
  Components::liftBy(0); // hold wherever the lift is now
}

void enterMode(Mode next) {
  mode = next;
  Components::liftRaw(0); // a held lift stick can't carry into the next mode
  // A press from the old mode can't count as half of a double tap in this one
  autoTaps = DoubleTap{};
  driverTaps = DoubleTap{};
  pidTaps = DoubleTap{};
  if (next == Mode::Auto) {
    // Driver control stops running here, so clear its last command
    Macro::cancel();
    chassis.tank(0, 0, true);
    Components::lift(0);
    Components::clawIntake(false);
    autoState = AutoState::Ready;
    wallReadFailed = false;
    startAllowedAt = pros::millis() + DOUBLE_TAP_MS;
  } else if (next == Mode::Pid) {
    PidTune::open();
  }
  Interface::setMode(modeLabel(next));
  CtrlScreen::rumble(".");
}

void stopAuton(Mode after) {
  // The next mode starts once the auton has actually exited
  AutonRun::cancel();
  stopping = true;
  nextMode = after;
}

void changeMode(Mode next) {
  if (AutonRun::running()) {
    stopAuton(next);
    return;
  }
  if (mode == Mode::Pid) {
    PidTune::close();
  }
  enterMode(next);
}

void pickSide(Settings::Side next) {
  if (Settings::side() != next) {
    Settings::setSide(next);
    wallReadFailed = false;
  }
}

void startAuton() {
  // The robot was probably placed back by hand, so read the walls every start.
  // If that fails, warn and wait. Another tap of A runs it anyway
  if (!wallReadFailed && !Settings::readPosition()) {
    wallReadFailed = true;
    startAllowedAt = pros::millis() + DOUBLE_TAP_MS;
    CtrlScreen::rumble("---");
    return;
  }
  wallReadFailed = false;
  AutonRun::start();
  autoState = AutoState::Running;
  CtrlScreen::rumble(".");
}

void updateAuto(const Controls &driver) {
  if (autoState == AutoState::Running) {
    if (driver.autoMode.newPress()) {
      stopAuton(Mode::Auto);
    }
    return;
  }

  if (driver.menuLeft.newPress()) {
    pickSide(Settings::Side::Left);
  } else if (driver.menuRight.newPress()) {
    pickSide(Settings::Side::Right);
  }
  // Up and down switch between the match and skills autons
  if (driver.menuUp.newPress() || driver.menuDown.newPress()) {
    Settings::setRoutine(Settings::routine() == Settings::Routine::Match
                             ? Settings::Routine::Skills
                             : Settings::Routine::Match);
  }
  if (driver.autoMode.newPress() &&
      static_cast<std::int32_t>(pros::millis() - startAllowedAt) >= 0) {
    startAuton();
  }
}

void drawAuto() {
  const bool running = autoState == AutoState::Running;
  char line[32];
  // Like "A:MATCH LEFT", the A: marks auto mode
  std::snprintf(line, sizeof(line), "A:%s %s",
                Settings::routineName(Settings::routine()), sideName());
  CtrlScreen::write(0, line);

  const char *state = "READY";
  if (stopping) {
    state = "STOPPING";
  } else if (running) {
    state = "RUN";
  } else if (autoState == AutoState::Done) {
    state = "DONE";
  } else if (autoState == AutoState::Stopped) {
    state = "STOPPED";
  }
  const double seconds =
      autoState == AutoState::Ready ? 0.0 : AutonRun::elapsedMs() / 1000.0;
  std::snprintf(line, sizeof(line), "%-8s %4.1fs", state, seconds);
  CtrlScreen::write(1, line);

  if (stopping) {
    CtrlScreen::write(2, "");
  } else if (running) {
    CtrlScreen::write(2, "A: stop");
  } else if (wallReadFailed) {
    CtrlScreen::write(2, "BAD POS A=run");
  } else {
    CtrlScreen::write(2, "A run ^v <>");
  }
}

void drawStatus() {
  if (mode == Mode::Pid) {
    return; // the tuner draws its own status
  }
  if (mode == Mode::Auto) {
    drawAuto();
    return;
  }
  char line[32];
  std::snprintf(line, sizeof(line), "TEST: %s", modeLabel(mode));
  CtrlScreen::write(0, line);
  std::snprintf(line, sizeof(line), "%s %s", sideName(),
                Settings::positionOk() ? "Pos OK" : "CHECK POS");
  CtrlScreen::write(1, line);
  CtrlScreen::write(2, "AA auto YY PID");
}

} // namespace

void begin() {
  Interface::setMode(modeLabel(mode));
  Settings::open();
}

void update(const Controls &driver) {
  CtrlScreen::update();
  if (Settings::update(driver)) {
    return; // the side menu has the controls
  }
  AutonRun::update();

  const bool autoDouble = autoTaps.update(driver.autoMode);
  const bool driverDouble = driverTaps.update(driver.driverMode);
  const bool pidDouble = pidTaps.update(driver.pidMode);

  if (stopping) {
    if (driverDouble) {
      nextMode = Mode::Driver;
    } else if (pidDouble) {
      nextMode = Mode::Pid;
    }
    if (AutonRun::running()) {
      drawStatus();
      return;
    }
    stopping = false;
    haltRobot();
    autoState = AutoState::Stopped;
    if (nextMode != Mode::Auto) {
      enterMode(nextMode);
    } else {
      startAllowedAt = pros::millis() + DOUBLE_TAP_MS;
      CtrlScreen::rumble(".");
    }
    drawStatus();
    return;
  }

  // The auton finished on its own, stay in auto mode to run it again
  if (autoState == AutoState::Running && !AutonRun::running()) {
    haltRobot();
    autoState = AutoState::Done;
    startAllowedAt = pros::millis() + DOUBLE_TAP_MS;
    CtrlScreen::rumble("..");
  }

  Mode wanted = mode;
  if (driverDouble) {
    wanted = Mode::Driver;
  } else if (pidDouble) {
    wanted = Mode::Pid;
  } else if (mode == Mode::Driver && autoDouble) {
    wanted = Mode::Auto;
  }
  if (wanted != mode) {
    changeMode(wanted);
    return;
  }

  if (mode == Mode::Driver) {
    Controller::execute(driver);
  } else if (mode == Mode::Auto) {
    updateAuto(driver);
  } else {
    PidTune::update(driver);
  }
  drawStatus();
}

} // namespace TestMode
