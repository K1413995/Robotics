#pragma once

// The controller screen: 3 lines of 15 characters
namespace CtrlScreen {
// Only sends a line if its text changed
void write(int line, const char *text);
// Pattern uses . for short and - for long
void rumble(const char *pattern);
// Call every loop, sends one update at a time because the screen is slow
void update();
} // namespace CtrlScreen
