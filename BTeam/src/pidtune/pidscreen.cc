// PID TUNER SCREEN
// Gains on the left, live graph and results on the right
#include "pidscreen.hpp"
#include "liblvgl/lvgl.h"
#include "pidstate.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace PidTune {
namespace {

// How often the screen updates, in milliseconds
constexpr int REFRESH_MS = 60;

// COLORS, matching the main brain screen
constexpr std::uint32_t BACKGROUND = 0x0B1220;
constexpr std::uint32_t SELECTED = 0x3432C7;
constexpr std::uint32_t PANEL = 0x182235;
constexpr std::uint32_t TRACK = 0x2B3750;
constexpr std::uint32_t WHITE = 0xFFFFFF;
constexpr std::uint32_t MUTED = 0x9AA6BA;
constexpr std::uint32_t GREEN = 0x34D399;
constexpr std::uint32_t YELLOW = 0xFBBF24;
constexpr std::uint32_t RED = 0xF87171;
constexpr std::uint32_t BLUE = 0x60A5FA;

// LAYOUT, the brain's drawing area is 480 x 240
constexpr int LEFT_X = 4;
constexpr int LEFT_WIDTH = 176;
constexpr int RIGHT_X = 184;
constexpr int RIGHT_WIDTH = 292;
constexpr int PANEL_Y = 4;
constexpr int PANEL_HEIGHT = 232;
constexpr int ROW_WIDTH = 168;
constexpr int ROW_HEIGHT = 20;
constexpr int DIGIT_WIDTH = 15;
constexpr int DIGIT_START = 44;
constexpr int DOT_WIDTH = 7;
constexpr int ACTION_WIDTH = 82;
constexpr int ACTION_HEIGHT = 26;
constexpr int GRAPH_WIDTH = 280;
constexpr int GRAPH_HEIGHT = 130;
constexpr int METRIC_X[] = {8, 104, 200};

struct GainRow {
  lv_obj_t *box;
  lv_obj_t *digits[DIGIT_COUNT];
  int shownValue;
};

GainRow gainRows[GAIN_COUNT];
lv_obj_t *actionBoxes[ACTION_COUNT];
lv_obj_t *screen = nullptr;
lv_obj_t *previousScreen = nullptr;
bool shown = false;

lv_obj_t *titleText = nullptr;
lv_obj_t *statusText = nullptr;
lv_obj_t *hintText = nullptr;
lv_obj_t *metricText[3];
lv_obj_t *worstText[3];
lv_obj_t *graph = nullptr;
lv_chart_series_t *targetLine = nullptr;
lv_chart_series_t *actualLine = nullptr;

int shownRow = -1;
int shownDigit = -1;
int shownEditing = -1;
int shownEpoch = -1;
int drawnPoints = 0;
int shownPhase = -1;
int shownHint = -1;
int shownTitle = -1;
int shownMetric[3] = {-1, -1, -1};
int shownWorst[3] = {-1, -1, -1};

lv_obj_t *makeBox(lv_obj_t *parent, int x, int y, int width, int height,
                  std::uint32_t fill, int radius) {
  lv_obj_t *box = lv_obj_create(parent);
  lv_obj_remove_style_all(box);
  lv_obj_set_pos(box, x, y);
  lv_obj_set_size(box, width, height);
  lv_obj_set_style_bg_color(box, lv_color_hex(fill), 0);
  lv_obj_set_style_bg_opa(box, LV_OPA_COVER, 0);
  lv_obj_set_style_radius(box, radius, 0);
  lv_obj_remove_flag(box, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(box, LV_OBJ_FLAG_CLICKABLE);
  return box;
}

lv_obj_t *makeText(lv_obj_t *parent, const lv_font_t *font, std::uint32_t color,
                   const char *text, lv_align_t align, int x, int y) {
  lv_obj_t *label = lv_label_create(parent);
  lv_obj_set_style_text_font(label, font, 0);
  lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
  lv_label_set_text(label, text);
  lv_obj_align(label, align, x, y);
  return label;
}

void addGainRow(lv_obj_t *panel, int gain, const char *name, int y) {
  GainRow &row = gainRows[gain];
  row.box = makeBox(panel, 4, y, ROW_WIDTH, ROW_HEIGHT, PANEL, 3);
  row.shownValue = -1;
  makeText(row.box, &lv_font_montserrat_12, WHITE, name, LV_ALIGN_LEFT_MID, 6,
           0);

  int x = DIGIT_START;
  for (int digit = 0; digit < DIGIT_COUNT; digit++) {
    if (digit == 3) {
      makeText(row.box, &lv_font_montserrat_14, WHITE, ".", LV_ALIGN_LEFT_MID,
               x + 1, 0);
      x += DOT_WIDTH;
    }
    lv_obj_t *label = lv_label_create(row.box);
    lv_obj_set_pos(label, x, 1);
    lv_obj_set_size(label, DIGIT_WIDTH, ROW_HEIGHT - 2);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(label, lv_color_hex(WHITE), 0);
    lv_obj_set_style_text_align(label, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_bg_color(label, lv_color_hex(YELLOW), 0);
    lv_obj_set_style_bg_opa(label, LV_OPA_TRANSP, 0);
    lv_obj_set_style_radius(label, 2, 0);
    lv_label_set_text(label, "0");
    row.digits[digit] = label;
    x += DIGIT_WIDTH;
  }
}

void addAction(lv_obj_t *panel, int index, const char *name, int x, int y) {
  actionBoxes[index] =
      makeBox(panel, x, y, ACTION_WIDTH, ACTION_HEIGHT, TRACK, 4);
  makeText(actionBoxes[index], &lv_font_montserrat_12, WHITE, name,
           LV_ALIGN_CENTER, 0, 0);
}

void buildLeftPanel() {
  lv_obj_t *panel =
      makeBox(screen, LEFT_X, PANEL_Y, LEFT_WIDTH, PANEL_HEIGHT, PANEL, 6);
  makeText(panel, &lv_font_montserrat_12, MUTED, "DRIVE", LV_ALIGN_TOP_LEFT, 10,
           4);
  addGainRow(panel, DRIVE_KP, "kP", 20);
  addGainRow(panel, DRIVE_KI, "kI", 41);
  addGainRow(panel, DRIVE_KD, "kD", 62);
  makeText(panel, &lv_font_montserrat_12, MUTED, "STEER", LV_ALIGN_TOP_LEFT, 10,
           87);
  addGainRow(panel, STEER_KP, "kP", 103);
  addGainRow(panel, STEER_KI, "kI", 124);
  addGainRow(panel, STEER_KD, "kD", 145);
  addAction(panel, 0, "RUN DRIVE", 4, 171);
  addAction(panel, 1, "RUN TURN", 90, 171);
  addAction(panel, 2, "RESET", 4, 201);
  addAction(panel, 3, "EXIT", 90, 201);
}

void buildRightPanel() {
  lv_obj_t *panel =
      makeBox(screen, RIGHT_X, PANEL_Y, RIGHT_WIDTH, PANEL_HEIGHT, PANEL, 6);
  titleText = makeText(panel, &lv_font_montserrat_14, WHITE, "PID TUNER",
                       LV_ALIGN_TOP_LEFT, 8, 5);
  statusText = makeText(panel, &lv_font_montserrat_12, MUTED, "READY",
                        LV_ALIGN_TOP_RIGHT, -8, 6);

  graph = lv_chart_create(panel);
  lv_obj_set_pos(graph, 6, 26);
  lv_obj_set_size(graph, GRAPH_WIDTH, GRAPH_HEIGHT);
  lv_chart_set_type(graph, LV_CHART_TYPE_LINE);
  lv_chart_set_point_count(graph, GRAPH_POINTS);
  lv_chart_set_div_line_count(graph, 5, 7);
  lv_chart_set_range(graph, LV_CHART_AXIS_PRIMARY_Y, -10, 110);
  lv_obj_remove_flag(graph, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_remove_flag(graph, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_bg_color(graph, lv_color_hex(BACKGROUND), 0);
  lv_obj_set_style_border_width(graph, 0, 0);
  lv_obj_set_style_radius(graph, 3, 0);
  lv_obj_set_style_pad_all(graph, 2, 0);
  lv_obj_set_style_line_color(graph, lv_color_hex(TRACK), LV_PART_MAIN);
  lv_obj_set_style_line_width(graph, 1, LV_PART_MAIN);
  lv_obj_set_style_line_width(graph, 2, LV_PART_ITEMS);
  lv_obj_set_style_size(graph, 0, 0, LV_PART_INDICATOR);
  targetLine =
      lv_chart_add_series(graph, lv_color_hex(YELLOW), LV_CHART_AXIS_PRIMARY_Y);
  actualLine =
      lv_chart_add_series(graph, lv_color_hex(BLUE), LV_CHART_AXIS_PRIMARY_Y);
  lv_chart_set_all_value(graph, targetLine, LV_CHART_POINT_NONE);
  lv_chart_set_all_value(graph, actualLine, LV_CHART_POINT_NONE);

  const char *captions[] = {"OVERSHOOT", "FINAL ERROR", "SETTLE"};
  for (int i = 0; i < 3; i++) {
    makeText(panel, &lv_font_montserrat_10, MUTED, captions[i],
             LV_ALIGN_TOP_LEFT, METRIC_X[i], 159);
    metricText[i] = makeText(panel, &lv_font_montserrat_16, WHITE, "--",
                             LV_ALIGN_TOP_LEFT, METRIC_X[i], 171);
    worstText[i] = makeText(panel, &lv_font_montserrat_10, MUTED, "worst --",
                            LV_ALIGN_TOP_LEFT, METRIC_X[i], 192);
  }
  hintText = makeText(panel, &lv_font_montserrat_12, MUTED, "",
                      LV_ALIGN_TOP_LEFT, 8, 210);
}

void buildScreen() {
  screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(screen, lv_color_hex(BACKGROUND), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
  lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
  buildLeftPanel();
  buildRightPanel();
}

void drawGains(const View &v) {
  for (int gain = 0; gain < GAIN_COUNT; gain++) {
    const int value = v.gains[gain].load();
    GainRow &row = gainRows[gain];
    if (value == row.shownValue) {
      continue;
    }
    row.shownValue = value;
    int remaining = value;
    for (int digit = DIGIT_COUNT - 1; digit >= 0; digit--) {
      lv_label_set_text_fmt(row.digits[digit], "%d", remaining % 10);
      remaining /= 10;
    }
  }
}

void drawSelection(const View &v) {
  const int row = v.row.load();
  const int digit = v.digit.load();
  const int editing = v.editing.load() ? 1 : 0;
  if (row == shownRow && digit == shownDigit && editing == shownEditing) {
    return;
  }
  shownRow = row;
  shownDigit = digit;
  shownEditing = editing;

  for (int gain = 0; gain < GAIN_COUNT; gain++) {
    lv_obj_set_style_bg_color(gainRows[gain].box,
                              lv_color_hex(gain == row ? SELECTED : PANEL), 0);
    for (int d = 0; d < DIGIT_COUNT; d++) {
      const bool active = editing && gain == row && d == digit;
      lv_obj_set_style_bg_opa(gainRows[gain].digits[d],
                              active ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
      lv_obj_set_style_text_color(gainRows[gain].digits[d],
                                  lv_color_hex(active ? BACKGROUND : WHITE), 0);
    }
  }
  for (int i = 0; i < ACTION_COUNT; i++) {
    lv_obj_set_style_bg_color(
        actionBoxes[i], lv_color_hex(row == GAIN_COUNT + i ? SELECTED : TRACK),
        0);
  }
}

void drawStatus(const View &v) {
  const int phase = v.phase.load();
  if (phase != shownPhase) {
    shownPhase = phase;
    const char *names[] = {"READY", "RUNNING", "DONE", "STOPPED"};
    const std::uint32_t colors[] = {MUTED, YELLOW, GREEN, RED};
    lv_label_set_text(statusText, names[phase]);
    lv_obj_set_style_text_color(statusText, lv_color_hex(colors[phase]), 0);
    shownHint = -1;
  }

  const int hint = v.hint.load();
  if (hint != shownHint) {
    shownHint = hint;
    const char *hints[] = {"Up/Down select, L1 enter",
                           "Overshoots: raise kD or lower kP",
                           "Stops short: raise kP", "Slow to settle: raise kP",
                           "Looks good, write these down"};
    const std::uint32_t colors[] = {MUTED, YELLOW, YELLOW, YELLOW, GREEN};
    lv_label_set_text(hintText, phase == PHASE_RUNNING
                                    ? "B or a stick stops the test"
                                    : hints[hint]);
    lv_obj_set_style_text_color(
        hintText, lv_color_hex(phase == PHASE_RUNNING ? MUTED : colors[hint]),
        0);
  }

  if (phase == PHASE_READY) {
    return;
  }
  const bool steer = v.steerTest.load();
  const int number = v.moveNumber.load();
  const int size = static_cast<int>(std::lround(v.moveSize.load()));
  const int title = (steer ? 100000 : 0) + number * 1000 + size;
  if (title != shownTitle) {
    shownTitle = title;
    lv_label_set_text_fmt(titleText, "%s %d%s  %d/%d", steer ? "TURN" : "DRIVE",
                          size, steer ? "\xC2\xB0" : " in", number,
                          v.moveCount.load());
  }
}

void setLength(lv_obj_t *label, int &shownValue, float value, bool steer,
               const char *prefix) {
  const int tenths = static_cast<int>(std::lround(value * 10));
  if (tenths == shownValue) {
    return;
  }
  shownValue = tenths;
  lv_label_set_text_fmt(label, "%s%d.%d%s", prefix, tenths / 10, tenths % 10,
                        steer ? "\xC2\xB0" : " in");
}

void setSeconds(lv_obj_t *label, int &shownValue, float value,
                const char *prefix) {
  const int hundredths = static_cast<int>(std::lround(value * 100));
  if (hundredths == shownValue) {
    return;
  }
  shownValue = hundredths;
  lv_label_set_text_fmt(label, "%s%d.%02ds", prefix, hundredths / 100,
                        hundredths % 100);
}

void drawMetrics(const View &v) {
  if (v.phase.load() == PHASE_READY) {
    return;
  }
  const bool steer = v.steerTest.load();
  setLength(metricText[0], shownMetric[0], v.overshoot.load(), steer, "");
  setLength(metricText[1], shownMetric[1], v.finalError.load(), steer, "");
  setSeconds(metricText[2], shownMetric[2], v.settleTime.load(), "");
  setLength(worstText[0], shownWorst[0], v.worstOvershoot.load(), steer,
            "worst ");
  setLength(worstText[1], shownWorst[1], v.worstError.load(), steer, "worst ");
  setSeconds(worstText[2], shownWorst[2], v.worstSettle.load(), "worst ");
}

// Blue is where the robot went, yellow is the target
void drawGraph(const View &v) {
  const int epoch = v.graphEpoch.load();
  bool changed = false;
  if (epoch != shownEpoch) {
    shownEpoch = epoch;
    drawnPoints = 0;
    const int target = static_cast<int>(std::lround(v.graphTarget.load() * 10));
    const int margin = std::max(target / 4, 5);
    lv_chart_set_range(graph, LV_CHART_AXIS_PRIMARY_Y, -margin,
                       target + margin);
    lv_chart_set_all_value(graph, targetLine, target);
    lv_chart_set_all_value(graph, actualLine, LV_CHART_POINT_NONE);
    changed = true;
  }

  const int count = std::min(v.graphCount.load(), GRAPH_POINTS);
  for (; drawnPoints < count; drawnPoints++) {
    lv_chart_set_value_by_id(
        graph, actualLine, drawnPoints,
        static_cast<int>(std::lround(v.graph[drawnPoints] * 10)));
    changed = true;
  }
  if (changed) {
    lv_chart_refresh(graph);
  }
}

void refresh(lv_timer_t *) {
  const View &v = view();
  const bool open = v.open.load();

  if (open && !shown) {
    if (screen == nullptr) {
      buildScreen();
    }
    previousScreen = lv_screen_active();
    lv_screen_load(screen);
    shown = true;
  } else if (!open && shown) {
    lv_screen_load(previousScreen);
    shown = false;
  }

  if (shown) {
    drawGains(v);
    drawSelection(v);
    drawStatus(v);
    drawMetrics(v);
    if (v.phase.load() != PHASE_READY) {
      drawGraph(v);
    }
  }
}

} // namespace

void startScreen() { lv_timer_create(refresh, REFRESH_MS, nullptr); }

} // namespace PidTune
