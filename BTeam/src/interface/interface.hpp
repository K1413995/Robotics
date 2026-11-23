#pragma once

namespace Interface {
// Draws motor temperatures and the lift rotation sensor on the brain
void start();

// Test code: shows the mode in the header in place of the team number.
// The text must stay valid, so pass a string literal
void setMode(const char *label);
} // namespace Interface
