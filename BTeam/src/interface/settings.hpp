#pragma once

#include "../controller/controls.hpp"

namespace Settings {
enum class Side { Left, Right };
enum class Routine { Match, Skills };

// The auton side picked in the menu
Side side();
void setSide(Side next);

// The auton picked in the menu
Routine routine();
void setRoutine(Routine next);
const char *routineName(Routine which);

// Reads the walls for the chosen side. True if both X and Y were found
bool readPosition();
// The result of the last wall reading
bool positionOk();

// Opens the pre-match menu on the controller screen
void open();

// Runs the menu. Returns true while the robot's controls should stay locked
bool update(const Controls &driver);
} // namespace Settings
