// POSITION
// Uses the distance sensors and heading to find the robot on the field.
// Field (0, 0) is the center. Heading 0 faces +Y and turns clockwise (LemLib).
#include "position.hpp"
#include "../hardware/hardware.hpp"
#include "../lemlib/drivetrain.hpp"
#include "auton.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>

namespace Position {
namespace {

constexpr double MM_PER_INCH = 25.4;
constexpr double DEG_TO_RAD = 3.14159265358979323846 / 180.0;

struct Sensor {
  pros::Distance *device;
  AutonConfig::SensorOffset offset;
  double mountAngle; // direction the sensor faces, relative to the robot
  bool active;
  std::array<std::int32_t, AutonConfig::SENSOR_SAMPLES> samples{};
  int count = 0;
};

// One axis can be seen by two sensors facing opposite walls
struct Axis {
  double value = 0;
  double distance = 0;
  int count = 0;
  bool conflict = false;

  void add(double newValue, double newDistance) {
    if (count > 0 &&
        std::fabs(value - newValue) > AutonConfig::SENSOR_AGREE_TOLERANCE) {
      conflict = true;
    }
    // the closer sensor is more accurate
    if (count == 0 || newDistance < distance) {
      value = newValue;
      distance = newDistance;
    }
    count++;
  }

  bool good() const { return count > 0 && !conflict; }
};

// One sensor's result, before checking it hit the wall we think it did
struct Reading {
  bool facesXWall;
  double value;         // robot X or Y from this sensor
  double distance;      // inches to the wall
  double sideTravel;    // how far the beam drifts along the wall
  double hitFromCenter; // where it lands along the wall, from robot center
};

bool goodReading(pros::Distance &sensor, std::int32_t mm) {
  // this also rejects 9999 (nothing seen) and PROS_ERR (unplugged)
  if (mm <= 0 || mm > AutonConfig::MAX_SENSOR_RANGE_MM) {
    return false;
  }
  // confidence only exists past 200 mm
  return mm <= 200 ||
         sensor.get_confidence() >= AutonConfig::MIN_SENSOR_CONFIDENCE;
}

Result measure(float heading, const std::string &side) {
  Result result;
  result.heading = heading;

  const bool both = side == "both";
  const bool left = both || side == "left";
  const bool right = both || side == "right";
  if (!left && !right) {
    return result; // unknown side name
  }

  std::array<Sensor, 3> sensors{
      Sensor{&Hardware::leftDistance, AutonConfig::LEFT_SENSOR_OFFSET, -90,
             left},
      Sensor{&Hardware::rightDistance, AutonConfig::RIGHT_SENSOR_OFFSET, 90,
             right},
      Sensor{&Hardware::backDistance, AutonConfig::BACK_SENSOR_OFFSET, 180,
             true},
  };

  // read every sensor at the same time so the check stays quick
  for (int i = 0; i < AutonConfig::SENSOR_SAMPLES; i++) {
    for (Sensor &sensor : sensors) {
      if (!sensor.active) {
        continue;
      }
      const std::int32_t mm = sensor.device->get_distance();
      if (goodReading(*sensor.device, mm)) {
        sensor.samples[sensor.count++] = mm;
      }
    }
    if (i < AutonConfig::SENSOR_SAMPLES - 1) {
      pros::delay(AutonConfig::SENSOR_SAMPLE_DELAY);
    }
  }

  const double theta = heading * DEG_TO_RAD;
  const double squareLimit = std::cos(AutonConfig::MAX_WALL_ANGLE * DEG_TO_RAD);
  std::array<Reading, 3> readings{};
  int readingCount = 0;

  for (Sensor &sensor : sensors) {
    // need most samples to be good, then the median ignores random spikes
    if (!sensor.active || sensor.count <= AutonConfig::SENSOR_SAMPLES / 2) {
      continue;
    }
    std::sort(sensor.samples.begin(), sensor.samples.begin() + sensor.count);
    const double distance = sensor.samples[sensor.count / 2] / MM_PER_INCH;

    // direction the sensor is pointing on the field
    const double ray = (heading + sensor.mountAngle) * DEG_TO_RAD;
    const double dirX = std::sin(ray);
    const double dirY = std::cos(ray);

    // where the sensor is compared to the robot's center, on the field
    const double offsetX = sensor.offset.right * std::cos(theta) +
                           sensor.offset.forward * std::sin(theta);
    const double offsetY = -sensor.offset.right * std::sin(theta) +
                           sensor.offset.forward * std::cos(theta);

    // wall position - distance to wall - sensor offset = robot center
    if (std::fabs(dirX) >= squareLimit) {
      const double wall = std::copysign(AutonConfig::FIELD_WIDTH / 2, dirX);
      readings[readingCount++] =
          Reading{true, wall - distance * dirX - offsetX, distance,
                  std::fabs(distance * dirY), offsetY + distance * dirY};
    } else if (std::fabs(dirY) >= squareLimit) {
      const double wall = std::copysign(AutonConfig::FIELD_LENGTH / 2, dirY);
      readings[readingCount++] =
          Reading{false, wall - distance * dirY - offsetY, distance,
                  std::fabs(distance * dirX), offsetX + distance * dirX};
    }
  }

  // A short or square beam can't reach the wall beside the one it faces
  Axis xAxis;
  Axis yAxis;
  for (int i = 0; i < readingCount; i++) {
    const Reading &reading = readings[i];
    if (reading.sideTravel <= AutonConfig::MIN_SIDE_CLEARANCE) {
      (reading.facesXWall ? xAxis : yAxis).add(reading.value, reading.distance);
    }
  }

  // A long tilted beam only counts if the other axis proves it hit the right
  // wall. If it really hit the wall beside it, it lands right at the corner.
  const Axis trustedX = xAxis;
  const Axis trustedY = yAxis;
  for (int i = 0; i < readingCount; i++) {
    const Reading &reading = readings[i];
    if (reading.sideTravel <= AutonConfig::MIN_SIDE_CLEARANCE) {
      continue;
    }
    const Axis &other = reading.facesXWall ? trustedY : trustedX;
    if (!other.good()) {
      continue;
    }
    const double halfWall = (reading.facesXWall ? AutonConfig::FIELD_LENGTH
                                                : AutonConfig::FIELD_WIDTH) /
                            2;
    if (std::fabs(other.value + reading.hitFromCenter) <=
        halfWall - AutonConfig::WALL_HIT_MARGIN) {
      (reading.facesXWall ? xAxis : yAxis).add(reading.value, reading.distance);
    }
  }

  if (xAxis.good()) {
    result.x = xAxis.value;
    result.hasX = true;
  }
  if (yAxis.good()) {
    result.y = yAxis.value;
    result.hasY = true;
  }
  return result;
}

} // namespace

Result getPos(const std::string &side) {
  return measure(chassis.getPose().theta, side);
}

bool setPos(float heading, const std::string &side) {
  const Result result = measure(heading, side);
  const lemlib::Pose current = chassis.getPose();
  chassis.setPose(result.hasX ? result.x : current.x,
                  result.hasY ? result.y : current.y, heading);
  return result.valid();
}

bool setPos(const std::string &side) {
  return setPos(chassis.getPose().theta, side);
}

} // namespace Position
