#pragma once

#include <atomic>

// Values shared between the tuner and its brain screen
namespace PidTune {

constexpr int GAIN_COUNT = 6;
constexpr int DIGIT_COUNT = 5; // 000.00
constexpr int ACTION_COUNT = 4;
constexpr int GRAPH_POINTS = 150; // 3 seconds at one point every 20 ms

enum Row {
  DRIVE_KP,
  DRIVE_KI,
  DRIVE_KD,
  STEER_KP,
  STEER_KI,
  STEER_KD,
  RUN_DRIVE,
  RUN_STEER,
  RESET_GAINS,
  EXIT_TUNER,
  ROW_COUNT
};

enum Phase { PHASE_READY, PHASE_RUNNING, PHASE_DONE, PHASE_STOPPED };

enum Hint { HINT_NONE, HINT_OVERSHOOT, HINT_SHORT, HINT_SLOW, HINT_GOOD };

struct View {
  std::atomic<bool> open{false};
  std::atomic<int> gains[GAIN_COUNT]{}; // hundredths, so 1050 is 10.50
  std::atomic<int> row{DRIVE_KP};
  std::atomic<int> digit{2};
  std::atomic<bool> editing{false};

  std::atomic<int> phase{PHASE_READY};
  std::atomic<int> hint{HINT_NONE};
  std::atomic<bool> steerTest{false};
  std::atomic<int> moveNumber{0};
  std::atomic<int> moveCount{0};
  std::atomic<float> moveSize{0};

  std::atomic<float> overshoot{0};
  std::atomic<float> finalError{0};
  std::atomic<float> settleTime{0};
  std::atomic<float> worstOvershoot{0};
  std::atomic<float> worstError{0};
  std::atomic<float> worstSettle{0};

  std::atomic<int> graphEpoch{0}; // changes when a new move starts
  std::atomic<int> graphCount{0};
  std::atomic<float> graphTarget{0}; // real distance this move has to travel
  float graph[GRAPH_POINTS] = {};    // distance traveled toward the target
};

View &view();

} // namespace PidTune
