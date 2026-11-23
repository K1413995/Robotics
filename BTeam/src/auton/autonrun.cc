// AUTON RUNNER
// LemLib holds a lock during moves, so killing the task could freeze the
// drivetrain. Instead cancel asks the auton to stop, and it returns on its own
#include "autonrun.hpp"
#include "../components/components.hpp"
#include "auton.hpp"
#include "pros/rtos.hpp"

#include <atomic>

namespace AutonRun {
namespace {

std::atomic<bool> isRunning{false};
std::atomic<bool> stopRequested{false};
std::atomic<std::uint32_t> startedAt{0};
std::atomic<std::uint32_t> endedAt{0};

} // namespace

void start() {
  if (isRunning.exchange(true)) {
    return;
  }
  stopRequested = false;
  const std::uint32_t now = pros::millis();
  startedAt = now;
  endedAt = now;
  pros::Task task(
      [] {
        auton();
        endedAt = pros::millis();
        // Cleared here so a real match auton can never start out canceled
        stopRequested = false;
        isRunning = false;
      },
      "Test Auton");
}

void cancel() {
  if (isRunning.load()) {
    stopRequested = true;
    chassis.cancelAllMotions();
  }
}

void update() {
  if (stopRequested.load() && isRunning.load()) {
    chassis.cancelAllMotions();
  }
}

bool running() { return isRunning.load(); }

std::uint32_t elapsedMs() {
  const std::uint32_t end = isRunning.load() ? pros::millis() : endedAt.load();
  return end - startedAt.load();
}

bool canceled() { return stopRequested.load(); }

bool wait(std::uint32_t ms) {
  const std::uint32_t start = pros::millis();
  while (pros::millis() - start < ms) {
    if (canceled()) {
      return false;
    }
    pros::delay(10);
  }
  return !canceled();
}

} // namespace AutonRun
