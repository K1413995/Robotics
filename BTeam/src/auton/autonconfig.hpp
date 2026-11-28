#pragma once

// Everything you measure or tune for positioning is here.
// All lengths are in INCHES unless the name says otherwise.
namespace AutonConfig {

// FIELD COORDINATES (Jerry.io "VEX Gaming Positioning System")
// Always plan as blue, standing at our driver box and facing the enemy.
// (0, 0) = center of the field
// X: + toward our wall, - toward the enemy
// Y: + to our right, - to our left
// Heading (clockwise): 0 = +Y, 90 = +X (our wall), 180 = -Y, 270 = -X (enemy)

// FIELD
// Inside wall to wall, measured in inches. Remeasure at your event if needed.
// VEX field spec: 140.4 in.
constexpr double FIELD_WIDTH = 140.4;  // X direction, inches
constexpr double FIELD_LENGTH = 140.4; // Y direction, inches

// TRACKING WHEEL
// Its real rolling diameter, in inches. A "2 inch" omni is 2.125 on paper,
// but the real number decides how far the robot thinks it drove.
// To calibrate: put the robot at a tape mark, push it straight 200 cm by hand
// along a tape measure, and read how far the brain says it went.
//   new diameter = old diameter * real distance / brain distance
// e.g. brain says 206 cm for a real 200: 2.125 * 200 / 206 = 2.063
constexpr float TRACKING_WHEEL_DIAMETER = 2.125;

// DISTANCE SENSOR OFFSETS
// Measure from the robot's center of rotation to the FRONT FACE of each sensor.
// forward: + toward the front of the robot, - toward the back
// right:   + toward the right of the robot, - toward the left
struct SensorOffset {
  double forward; // inches
  double right;   // inches
};

// Any error here moves the whole auton: the start position is read from these.
// TODO: RIGHT is still 0, so a right side start reads Y off by the sensor's
// distance from the center. Measure it before running the right side
constexpr SensorOffset LEFT_SENSOR_OFFSET{4, -5.3}; // right should be negative
constexpr SensorOffset RIGHT_SENSOR_OFFSET{0, 0};   // right should be positive
constexpr SensorOffset BACK_SENSOR_OFFSET{-7.5,
                                          0}; // forward should be negative

// AUTON START
// Read at startup and again when the controller menu is confirmed.
// Heading of the robot when placed on the field, in degrees.
// 270 = facing the enemy
constexpr float START_HEADING = 270;
// The start side is picked in the controller menu, see interface/settings.cc

// SENSOR TUNING
// Readings taken per position check
constexpr int SENSOR_SAMPLES = 5;
// Milliseconds between readings
constexpr int SENSOR_SAMPLE_DELAY = 35;
// Ignore anything farther than this, in millimeters
constexpr int MAX_SENSOR_RANGE_MM = 2000;
// Minimum confidence from 0 to 63, only used past 200 mm
constexpr int MIN_SENSOR_CONFIDENCE = 30;
// Degrees a sensor can be off square from a wall
constexpr double MAX_WALL_ANGLE = 20;
// Inches two opposite sensors can disagree before both are ignored
constexpr double SENSOR_AGREE_TOLERANCE = 3;
// Inches: smallest gap a sensor can have to the wall beside the one it faces
constexpr double MIN_SIDE_CLEARANCE = 6;
// Inches: a tilted beam must land this far from the corner to be trusted
constexpr double WALL_HIT_MARGIN = 2;

} // namespace AutonConfig
