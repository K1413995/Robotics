#pragma once

#include "../interface/settings.hpp"

// The autons. Points are written for the LEFT side and flip for the right.
// Helpers are in autonhelpers.hpp, field and sensor settings in autonconfig.hpp
namespace Auton {
void match(Settings::Side side);
void skills(Settings::Side side);

// Runs the picked auton, this is what main.cc and the test code call
void run(Settings::Routine routine, Settings::Side side);

// Reads the walls for a side and sets the start pose.
// True if both X and Y were found
bool setStartPose(Settings::Side side);
} // namespace Auton
