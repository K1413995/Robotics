// LIFT PID
// Only runs when the lift has a target: liftTo, liftBy, or holding a spot
#include "liftpid.hpp"
#include "../components/liftcore.hpp"
#include "lemlib/pid.hpp"
#include "pidconfig.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <new>

namespace LiftPid {
namespace {

using namespace PidConfig::Lift;

// The tuner runs on another task, so new gains wait here for the lift task
std::atomic<float> nextP{KP};
std::atomic<float> nextI{KI};
std::atomic<float> nextD{KD};
std::atomic<bool> gainsChanged{false};

lemlib::PID pid(KP, KI, KD, INTEGRAL_RANGE, true);
double targetHeight = 0;
bool hasTarget = false;
bool settled = false;
double appliedPower = 0;

void startPid(double error) {
  pid.reset();
  pid.update(error); // sets the last error so kD doesn't kick on the start
  LiftCore::resetStall();
}

void stopMotors() {
  appliedPower = 0;
  LiftCore::setPower(0);
}

// Interpolates the power so the motors never jump
void driveSmooth(double wanted) {
  appliedPower += std::clamp(wanted - appliedPower, -POWER_STEP, POWER_STEP);
  int output = static_cast<int>(std::lround(appliedPower));
  if (output == 0) {
    output = wanted >= 0 ? 1 : -1; // 0 would snap on the HOLD brake mid-move
  }
  LiftCore::setPower(output);
}

bool finishMove() {
  settled = true;
  stopMotors();
  return true;
}

} // namespace

Gains gains() { return {nextP.load(), nextI.load(), nextD.load()}; }

void setGains(Gains next) {
  nextP = next.kP;
  nextI = next.kI;
  nextD = next.kD;
  gainsChanged = true;
}

void setTarget(double target, double height) {
  targetHeight = target;
  hasTarget = true;
  settled = false;
  startPid(target - height);
}

void clear() {
  hasTarget = false;
  appliedPower = 0;
}

bool run(double height) {
  // PID gains are const, so the object is rebuilt with the new values
  if (gainsChanged.exchange(false)) {
    pid.~PID();
    ::new (static_cast<void *>(&pid))
        lemlib::PID(nextP.load(), nextI.load(), nextD.load(), INTEGRAL_RANGE,
                    true);
    startPid(targetHeight - height);
  }

  if (!hasTarget) {
    stopMotors();
    return false;
  }

  const double error = targetHeight - height;
  if (settled) {
    if (std::fabs(error) <= DRIFT_LIMIT) {
      stopMotors();
      return false;
    }
    settled = false;
    startPid(error);
  }

  if (std::fabs(error) <= TOLERANCE) {
    return finishMove();
  }
  if (LiftCore::isStalled(height, PID_STALL_TIME)) {
    targetHeight = height; // blocked, so stay here instead of straining
    return finishMove();
  }

  double power = pid.update(error);
  if (error > 0 && power >= 0) {
    power = std::max<double>(power, MIN_UP_POWER);
  } else if (error < 0 && power <= 0) {
    power = std::min<double>(power, -MIN_DOWN_POWER);
  }
  power = std::clamp<double>(power, -PID_MAX_POWER, PID_MAX_POWER);

  if (LiftCore::roomToLimit(power > 0, height) <= 0) {
    stopMotors();
  } else {
    driveSmooth(power);
  }
  return false;
}

} // namespace LiftPid
