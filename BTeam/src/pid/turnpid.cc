// TURN PID
#include "turnpid.hpp"
#include "lemlib/pid.hpp"
#include "pidconfig.hpp"

#include <atomic>
#include <new>

namespace TurnPid {
namespace {

// The tuner runs on another task, so new gains wait here for the turn task
std::atomic<float> nextP{PidConfig::TURN.kP};
std::atomic<float> nextI{PidConfig::TURN.kI};
std::atomic<float> nextD{PidConfig::TURN.kD};
std::atomic<bool> gainsChanged{false};

lemlib::PID pid(PidConfig::TURN.kP, PidConfig::TURN.kI, PidConfig::TURN.kD,
                PidConfig::TURN.windup, true);

// PID gains are const, so the object is rebuilt with the new values
void applyNewGains() {
  if (gainsChanged.exchange(false)) {
    pid.~PID();
    ::new (static_cast<void *>(&pid))
        lemlib::PID(nextP.load(), nextI.load(), nextD.load(),
                    PidConfig::TURN.windup, true);
  }
}

} // namespace

void setGains(Gains next) {
  nextP = next.kP;
  nextI = next.kI;
  nextD = next.kD;
  gainsChanged = true;
}

void restart(double error) {
  applyNewGains();
  pid.reset();
  pid.update(error); // sets the last error so kD doesn't kick on the start
}

double update(double error) { return pid.update(error); }

} // namespace TurnPid
