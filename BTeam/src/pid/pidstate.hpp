#pragma once

#include <atomic>

// Values shared between the tuner and its brain screen
namespace PidTune {

constexpr int SYSTEM_COUNT = 3;
constexpr int TERM_COUNT = 3;
constexpr int DIGIT_COUNT = 5;    // 000.00
constexpr int RESET_ROW = 3;      // the row below kP, kI, kD
// The whole test goes on one graph. When it fills up, every other point is
// dropped so the rest of the test still fits
constexpr int GRAPH_POINTS = 230; // 2 pixels each across the graph

enum System { SYS_DRIVE, SYS_TURN, SYS_LIFT };
inline constexpr const char *SYSTEM_NAMES[SYSTEM_COUNT] = {"DRIVE", "TURN",
                                                           "LIFT"};
inline constexpr const char *TERM_NAMES[TERM_COUNT] = {"kP", "kI", "kD"};

enum Page { PAGE_GRAPH, PAGE_TUNE };

// Where the arrow keys are working on the tune page
enum Focus { FOCUS_TEST, FOCUS_DROPDOWN, FOCUS_VALUES, FOCUS_DIGIT };

enum Phase { PHASE_READY, PHASE_RUNNING, PHASE_DONE, PHASE_STOPPED };

enum Hint {
  HINT_NONE,
  HINT_OVERSHOOT,
  HINT_SHORT,
  HINT_SLOW,
  HINT_GOOD,
  HINT_NO_SENSOR
};

struct View {
  std::atomic<bool> open{false};
  // Hundredths, so 1050 is 10.50
  std::atomic<int> gains[SYSTEM_COUNT][TERM_COUNT]{};

  std::atomic<int> page{PAGE_GRAPH};
  std::atomic<int> system{SYS_DRIVE};
  std::atomic<int> focus{FOCUS_TEST};
  std::atomic<int> dropdownRow{SYS_DRIVE};
  std::atomic<int> valueRow{0};
  std::atomic<int> digit{2};

  std::atomic<int> phase{PHASE_READY};
  std::atomic<int> hint{HINT_NONE};
  std::atomic<int> testSystem{SYS_DRIVE};
  std::atomic<int> moveNumber{0};
  std::atomic<int> moveCount{0};
  std::atomic<float> moveSize{0};

  std::atomic<float> overshoot{0};
  std::atomic<float> finalError{0};
  std::atomic<float> settleTime{0};
  std::atomic<float> worstOvershoot{0};
  std::atomic<float> worstError{0};
  std::atomic<float> worstSettle{0};

  // Changes when the graph starts over or gets squeezed, so it redraws
  std::atomic<int> graphEpoch{0};
  std::atomic<int> graphCount{0};
  std::atomic<float> graphMax{0};  // the biggest target in this test
  float graph[GRAPH_POINTS] = {};  // distance from the start of the test
  float targets[GRAPH_POINTS] = {}; // where each move was headed
};

View &view();

} // namespace PidTune
