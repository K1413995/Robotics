#include "components.hpp"
#include "pros/rtos.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>

// lift brake mode is HOLD, in main.cc intialize()

// LIFT LIMITS, in degrees, using the angle shown on the brain screen
// Lowest angle the motors will drive down to
constexpr double LIFT_MIN_ANGLE = 5;
// Highest angle the motors will drive up to
constexpr double LIFT_MAX_ANGLE = 200;
// Use -1 if the angle drops as the lift rises
constexpr int LIFT_ANGLE_DIRECTION = 1;

// LIFT GEARING
// Green cartridge top speed in RPM, and the gears from the motor to the lift
constexpr double LIFT_MOTOR_RPM = 200;
constexpr double LIFT_GEAR_RATIO = 12.0 / 36.0;
// Lift top speed at power 127, in degrees per second (1 RPM = 6 deg/s)
constexpr double LIFT_FULL_SPEED = LIFT_MOTOR_RPM * LIFT_GEAR_RATIO * 6;

// DRIVER FEEL, speeds are degrees per second at the lift
// Milliseconds to smoothly build to full speed
// Lower feels snappier, higher makes short taps smaller
constexpr double LIFT_ACCEL_TIME = 300;
// Speed a press starts from
constexpr double LIFT_START_SPEED = 20;
// How hard it may slow down for a limit, in degrees per second^2
// Lower starts slowing earlier and lands softer
constexpr double LIFT_BOTTOM_DECEL = 1600;
constexpr double LIFT_TOP_DECEL = 3000;
// Speed the lift reaches a limit at
constexpr double LIFT_LANDING_SPEED = 10;
// How gently the last part of a stop fades out, lower is softer and longer
constexpr double LIFT_BOTTOM_EASE = 8;
constexpr double LIFT_TOP_EASE = 20;
// Seconds of travel to look ahead, covers the motors' reaction time
constexpr double LIFT_LOOK_AHEAD = 0.06;
// Most the commanded speed can change, in degrees per second^2
constexpr double LIFT_MAX_SPEED_CHANGE = 3500;

// LETTING GO
// Milliseconds to fade the speed to zero after letting go
// Higher is softer but travels farther after release
constexpr double LIFT_RELEASE_TIME = 50;
// Milliseconds to settle at zero speed before HOLD locks in
constexpr double LIFT_RELEASE_SETTLE = 100;

// FLOOR AND STALL PROTECTION
// The driver's floor check only runs this close to the bottom, in degrees
constexpr double LIFT_FLOOR_ZONE = 12;
// If the lift moves less than this many degrees...
constexpr double LIFT_STALL_MOVEMENT = 0.4;
// ...for this many milliseconds, it is blocked and the motors stop pushing
constexpr std::uint32_t LIFT_STALL_TIME = 150;
// Milliseconds a new press gets to start moving before the floor check runs
constexpr std::uint32_t LIFT_STALL_GRACE = 80;

// LIFT PID, used by liftTo and to recover if the lift gets pushed
// kP moves it, kD stops overshoot, kI adds power for heavier stacks
constexpr float LIFT_KP = 4;
constexpr float LIFT_KI = 0.2;
constexpr float LIFT_KD = 15;
// kI only builds up within this many degrees of the target
constexpr float LIFT_INTEGRAL_RANGE = 15;
// The PID gets longer than the driver before it counts as blocked,
// so kI has time to build enough power for a heavy stack
constexpr std::uint32_t LIFT_PID_STALL_TIME = 400;
// Strongest power the PID can use, out of 127
constexpr int LIFT_PID_MAX_POWER = 95;
// Weakest power that still moves the lift, only used while it is moving
constexpr int LIFT_MIN_UP_POWER = 25;
constexpr int LIFT_MIN_DOWN_POWER = 10;
// Most the PID's motor power can change every 10 ms
constexpr double LIFT_POWER_STEP = 10;
// Within this many degrees of the target, HOLD brake takes over
constexpr double LIFT_TOLERANCE = 1.5;
// While braked, the PID wakes up if the lift drifts farther than this
constexpr double LIFT_DRIFT_LIMIT = 4;
// The PID has no timer of its own, so it needs a steady loop
constexpr std::uint32_t LIFT_LOOP_MS = 10;

static constexpr int NO_REQUEST = INT32_MIN;

// Shared between the driver or auton code and the lift task
static std::atomic<int> manualPower{0};
static std::atomic<int> requestedHeight{NO_REQUEST}; // tenths of a degree
static std::atomic<int> rawPower{0};                 // set by macros
static std::atomic<bool> liftDone{true};

// Only the lift task uses these
static lemlib::PID liftPID(LIFT_KP, LIFT_KI, LIFT_KD, LIFT_INTEGRAL_RANGE,
                           true);
static double targetHeight = 0;
static bool hasTarget = false;
static bool settled = false;
static bool wasManual = false;
static bool motorsBraked = false;
static bool watchingStall = false;
static double stallHeight = 0;
static std::uint32_t stallTime = 0;
static int manualDirection = 0;
static std::uint32_t manualStart = 0;
static double manualBaseSpeed = 0;
static bool releasing = false;
static std::uint32_t releaseStart = 0;
static double releaseSpeed = 0;
static double profileSpeed = 0; // speed currently commanded, positive is up
static double appliedPower = 0; // PID power currently commanded

static double wrapDegrees(double degrees) {
  return std::fmod(std::fmod(degrees, 360.0) + 360.0, 360.0);
}

static double liftRange() {
  return wrapDegrees(LIFT_ANGLE_DIRECTION * (LIFT_MAX_ANGLE - LIFT_MIN_ANGLE));
}

// Degrees above the min angle
static double angleToHeight(double angle) {
  const double range = liftRange();
  double height = wrapDegrees(LIFT_ANGLE_DIRECTION * (angle - LIFT_MIN_ANGLE));
  // A little below the bottom should not look like almost a full circle up
  if (height > range + (360.0 - range) / 2) {
    height -= 360.0;
  }
  return height;
}

static double roomToLimit(bool goingUp, double height) {
  return goingUp ? liftRange() - height : height;
}

// S-curve between 0 and 1
static double smoothStep(double t) {
  t = std::clamp(t, 0.0, 1.0);
  return t * t * (3 - 2 * t);
}

static void setMotors(int power) {
  if (power != 0) {
    Hardware::liftMotors.move(power);
    motorsBraked = false;
  } else {
    appliedPower = 0;
    profileSpeed = 0;
    if (!motorsBraked) {
      // Brake once so HOLD keeps the spot where the lift stopped
      Hardware::liftMotors.brake();
      motorsBraked = true;
    }
  }
}

// The motor's built-in velocity control handles weight and friction
static void commandSpeed(double liftSpeed) {
  profileSpeed = liftSpeed;
  const double motorRpm = liftSpeed / (LIFT_GEAR_RATIO * 6);
  Hardware::liftMotors.move_velocity(static_cast<int>(std::lround(motorRpm)));
  motorsBraked = false;
}

// Interpolates the PID's power so the motors never jump
static void driveSmooth(double wanted) {
  appliedPower +=
      std::clamp(wanted - appliedPower, -LIFT_POWER_STEP, LIFT_POWER_STEP);
  int output = static_cast<int>(std::lround(appliedPower));
  if (output == 0) {
    output = wanted >= 0 ? 1 : -1; // 0 would snap on the HOLD brake mid-move
  }
  setMotors(output);
}

static bool isStalled(double height, std::uint32_t stallLimit) {
  const std::uint32_t now = pros::millis();
  if (!watchingStall || std::fabs(height - stallHeight) > LIFT_STALL_MOVEMENT) {
    watchingStall = true;
    stallHeight = height;
    stallTime = now;
    return false;
  }
  return now - stallTime >= stallLimit;
}

// Fastest the lift can go and still stop smoothly in the room left
static double stoppingSpeed(bool goingUp, double room) {
  const double decel = goingUp ? LIFT_TOP_DECEL : LIFT_BOTTOM_DECEL;
  const double ease = goingUp ? LIFT_TOP_EASE : LIFT_BOTTOM_EASE;
  const double usableRoom =
      std::max(room - std::fabs(profileSpeed) * LIFT_LOOK_AHEAD, 0.0);

  // Steady braking when far away, then a gentle fade for the last few degrees.
  // The two curves meet with the same slope, so there is no jolt between them
  const double easeRoom = decel / (ease * ease);
  const double curve = usableRoom <= easeRoom
                           ? ease * usableRoom
                           : std::sqrt(2 * decel * (usableRoom - easeRoom / 2));
  return LIFT_LANDING_SPEED + curve;
}

// Builds speed along an S-curve, and only slows down when a limit requires it,
// so the lift stays true to the driver's input
static void runDriverProfile(int power, double height) {
  const bool goingUp = power > 0;
  const double room = roomToLimit(goingUp, height);
  if (room <= 0) {
    setMotors(0);
    return;
  }

  const double topSpeed = std::abs(power) / 127.0 * LIFT_FULL_SPEED;
  const double base = std::min(topSpeed, manualBaseSpeed);
  const double elapsed = pros::millis() - manualStart;
  const double built =
      base + (topSpeed - base) * smoothStep(elapsed / LIFT_ACCEL_TIME);
  const double wanted = std::min(built, stoppingSpeed(goingUp, room));
  const double target = goingUp ? wanted : -wanted;

  const double maxChange = LIFT_MAX_SPEED_CHANGE * LIFT_LOOP_MS / 1000.0;
  commandSpeed(profileSpeed +
               std::clamp(target - profileSpeed, -maxChange, maxChange));
}

static void startPid(double error) {
  liftPID.reset();
  liftPID.update(error); // sets the last error so kD doesn't kick on the start
  watchingStall = false;
}

static void startTarget(double target, double height) {
  targetHeight = target;
  hasTarget = true;
  settled = false;
  startPid(target - height);
}

static void finishMove() {
  settled = true;
  liftDone = true;
  setMotors(0);
}

static void runPid(double height) {
  const double error = targetHeight - height;

  if (settled) {
    if (std::fabs(error) <= LIFT_DRIFT_LIMIT) {
      setMotors(0);
      return;
    }
    settled = false;
    startPid(error);
  }

  if (std::fabs(error) <= LIFT_TOLERANCE) {
    finishMove();
    return;
  }
  if (isStalled(height, LIFT_PID_STALL_TIME)) {
    targetHeight = height; // blocked, so stay here instead of straining
    finishMove();
    return;
  }

  double power = liftPID.update(error);
  if (error > 0 && power >= 0) {
    power = std::max<double>(power, LIFT_MIN_UP_POWER);
  } else if (error < 0 && power <= 0) {
    power = std::min<double>(power, -LIFT_MIN_DOWN_POWER);
  }
  power = std::clamp<double>(power, -LIFT_PID_MAX_POWER, LIFT_PID_MAX_POWER);
  if (roomToLimit(power > 0, height) <= 0) {
    setMotors(0);
  } else {
    driveSmooth(power);
  }
}

static void stepLift() {
  const std::int32_t angle = Hardware::liftRotation.get_angle();
  const int manual = manualPower.load();
  const std::uint32_t now = pros::millis();
  int request = NO_REQUEST;
  if (requestedHeight.load() != NO_REQUEST) {
    liftDone = false;
    request = requestedHeight.exchange(NO_REQUEST);
  }

  // Sensor unplugged: no PID or limits, but the driver can still use the lift
  if (angle == PROS_ERR) {
    hasTarget = false;
    releasing = false;
    profileSpeed = 0;
    wasManual = manual != 0;
    liftDone = true;
    setMotors(manual);
    return;
  }
  const double height = angleToHeight(angle / 100.0);

  // Driver is holding a button
  if (manual != 0) {
    const int direction = manual > 0 ? 1 : -1;
    if (!wasManual || direction != manualDirection) {
      manualDirection = direction;
      manualStart = now;
      // Pressing again mid-stop continues from the current speed
      manualBaseSpeed = std::max(LIFT_START_SPEED, direction * profileSpeed);
      watchingStall = false;
    }
    hasTarget = false;
    releasing = false;
    wasManual = true;
    liftDone = true;

    const bool starting = now - manualStart < LIFT_STALL_GRACE;
    if (manual > 0 || height > LIFT_FLOOR_ZONE || starting) {
      watchingStall = false;
    } else if (isStalled(height, LIFT_STALL_TIME)) {
      setMotors(0); // on the floor, stop pushing
      return;
    }
    runDriverProfile(manual, height);
    return;
  }

  // A macro is using raw motor power, with no ramp, slowing, or PID
  const int raw = rawPower.load();
  if (raw != 0) {
    hasTarget = false;
    releasing = false;
    wasManual = true; // hold wherever it stops once raw power ends
    manualDirection = 0;
    profileSpeed = 0;
    liftDone = false;
    setMotors(roomToLimit(raw > 0, height) > 0 ? raw : 0);
    return;
  }

  // Button released: ease the speed to zero along an S-curve
  if (wasManual) {
    wasManual = false;
    hasTarget = false;
    releasing = true;
    releaseStart = now;
    releaseSpeed = profileSpeed;
  }
  if (request != NO_REQUEST) {
    releasing = false;
    startTarget(request / 10.0, height);
  }
  if (releasing) {
    const double elapsed = now - releaseStart;
    if (elapsed < LIFT_RELEASE_TIME + LIFT_RELEASE_SETTLE &&
        roomToLimit(releaseSpeed > 0, height) > 0) {
      commandSpeed(releaseSpeed *
                   (1 - smoothStep(elapsed / LIFT_RELEASE_TIME)));
      return;
    }
    // Stopped, so HOLD can take over without a bounce
    releasing = false;
    setMotors(0);
    startTarget(height, height);
  }

  if (hasTarget) {
    runPid(height);
  } else {
    setMotors(0);
  }
}

void Components::startLift() {
  static pros::Task liftTask(
      [] {
        while (true) {
          stepLift();
          pros::delay(LIFT_LOOP_MS);
        }
      },
      "Lift");
}

// Driver control: positive is up, negative is down, 0 holds the current spot
void Components::lift(int voltage) { manualPower = voltage; }

// Moves to a brain-screen angle, staying inside the limits
void Components::liftTo(double angle) {
  const double height = std::clamp(angleToHeight(angle), 0.0, liftRange());
  liftDone = false;
  requestedHeight = static_cast<int>(std::lround(height * 10));
}

// Raw motor power for macros, 0 stops and holds wherever the lift ends up
void Components::liftRaw(int power) {
  if (power != 0) {
    liftDone = false;
  }
  rawPower = power;
}

// True once the lift has arrived at its target or gotten blocked
bool Components::liftSettled() {
  return requestedHeight.load() == NO_REQUEST && rawPower.load() == 0 &&
         liftDone.load();
}

// Waits until the lift arrives, gets blocked, or runs out of time
void Components::waitForLift(int timeoutMs) {
  const std::uint32_t start = pros::millis();
  while (!liftSettled() &&
         pros::millis() - start < static_cast<std::uint32_t>(timeoutMs)) {
    pros::delay(LIFT_LOOP_MS);
  }
}
