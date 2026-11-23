#pragma once

#include <string>

namespace Position {

struct Result {
  float x = 0;       // inches from field center
  float y = 0;       // inches from field center
  float heading = 0; // degrees
  bool hasX = false; // true if x came from a good sensor reading
  bool hasY = false; // true if y came from a good sensor reading

  bool valid() const { return hasX && hasY; }
};

// side can be "both", "left", or "right". The back sensor is always used.
// Use "left" or "right" in a corner to ignore the sensor facing into the field.
// These take about 150 ms, so stop the robot before calling them.

// Reads position using the current heading. Does not change the robot's pose.
Result getPos(const std::string &side = "both");

// Reads position and updates the robot's pose using the current heading.
// Returns true if both X and Y were found. A missing axis keeps its old value.
bool setPos(const std::string &side = "both");

// Same as above, but also sets the heading. Use this at the start of auton.
bool setPos(float heading, const std::string &side = "both");

} // namespace Position
