// PID TUNER SCREEN
// Two pages, Y swaps them: the live graph, and the gain editor
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
constexpr int PAGE_X = 4;
constexpr int PAGE_Y = 4;
constexpr int PAGE_WIDTH = 472;
constexpr int PAGE_HEIGHT = 232;
constexpr int GRAPH_WIDTH = 460;
constexpr int GRAPH_HEIGHT = 130;
constexpr int METRIC_X[] = {8, 160, 312};

constexpr int EDIT_X = 8;
constexpr int EDIT_WIDTH = 300;
constexpr int TEST_ROW_Y = 28;
constexpr int TEST_ROW_HEIGHT = 34;
constexpr int TERM_ROW_Y = 70;
constexpr int TERM_ROW_HEIGHT = 32;
constexpr int TERM_ROW_GAP = 36;
constexpr int RESET_ROW_Y = 180;
constexpr int RESET_ROW_HEIGHT = 30;
constexpr int DIGIT_START = 80;
constexpr int DIGIT_WIDTH = 24;
constexpr int DOT_WIDTH = 10;
constexpr int DROPDOWN_ROW_HEIGHT = 34;
constexpr int HELP_X = 318;
constexpr int HELP_WIDTH = 146;
constexpr int HELP_HEIGHT = 182;

constexpr const char *SYSTEM_TITLES[SYSTEM_COUNT] = {
    "DRIVE  forward/back", "TURN  steering", "LIFT"};

struct TermRow {
  lv_obj_t *box;
  lv_obj_t *digits[DIGIT_COUNT];
  int shownValue;
};

lv_obj_t *screen = nullptr;
lv_obj_t *previousScreen = nullptr;
bool shown = false;
lv_obj_t *graphPage = nullptr;
lv_obj_t *tunePage = nullptr;
int shownPage = -1;

// Graph page
lv_obj_t *titleText = nullptr;
lv_obj_t *statusText = nullptr;
lv_obj_t *gainText = nullptr;
lv_obj_t *hintText = nullptr;
lv_obj_t *metricText[3];
lv_obj_t *worstText[3];
lv_obj_t *graph = nullptr;
lv_chart_series_t *targetLine = nullptr;
lv_chart_series_t *actualLine = nullptr;
int shownEpoch = -1;
int drawnPoints = 0;
int shownPhase = -1;
int shownHint = -1;
int shownTitle = -1;
int shownMetric[3] = {-1, -1, -1};
int shownWorst[3] = {-1, -1, -1};
int shownGains[TERM_COUNT + 1] = {-1, -1, -1, -1};

// Tune page
lv_obj_t *testBox = nullptr;
lv_obj_t *testText = nullptr;
TermRow termRows[TERM_COUNT];
lv_obj_t *resetBox = nullptr;
lv_obj_t *helpText = nullptr;
lv_obj_t *dropdownBox = nullptr;
lv_obj_t *dropdownRows[SYSTEM_COUNT];
int shownSystem = -1;
int shownSelection = -1;

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

void buildGraphPage() {
  graphPage =
      makeBox(screen, PAGE_X, PAGE_Y, PAGE_WIDTH, PAGE_HEIGHT, PANEL, 6);
  titleText = makeText(graphPage, &lv_font_montserrat_14, WHITE, "PID GRAPH",
                       LV_ALIGN_TOP_LEFT, 8, 5);
  gainText = makeText(graphPage, &lv_font_montserrat_12, MUTED, "",
                      LV_ALIGN_TOP_MID, 30, 6);
  statusText = makeText(graphPage, &lv_font_montserrat_12, MUTED, "READY",
                        LV_ALIGN_TOP_RIGHT, -8, 6);

  graph = lv_chart_create(graphPage);
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
    makeText(graphPage, &lv_font_montserrat_10, MUTED, captions[i],
             LV_ALIGN_TOP_LEFT, METRIC_X[i], 159);
    metricText[i] = makeText(graphPage, &lv_font_montserrat_16, WHITE, "--",
                             LV_ALIGN_TOP_LEFT, METRIC_X[i], 171);
    worstText[i] = makeText(graphPage, &lv_font_montserrat_10, MUTED,
                            "worst --", LV_ALIGN_TOP_LEFT, METRIC_X[i], 192);
  }
  hintText = makeText(graphPage, &lv_font_montserrat_12, MUTED, "",
                      LV_ALIGN_TOP_LEFT, 8, 210);
}

void addTermRow(int term, int y) {
  TermRow &row = termRows[term];
  row.box = makeBox(tunePage, EDIT_X, y, EDIT_WIDTH, TERM_ROW_HEIGHT, TRACK, 4);
  row.shownValue = -1;
  makeText(row.box, &lv_font_montserrat_16, WHITE, TERM_NAMES[term],
           LV_ALIGN_LEFT_MID, 12, 0);

  int x = DIGIT_START;
  for (int digit = 0; digit < DIGIT_COUNT; digit++) {
    if (digit == 3) {
      makeText(row.box, &lv_font_montserrat_24, WHITE, ".", LV_ALIGN_LEFT_MID,
               x + 2, 0);
      x += DOT_WIDTH;
    }
    lv_obj_t *label = lv_label_create(row.box);
    lv_obj_set_pos(label, x, 2);
    lv_obj_set_size(label, DIGIT_WIDTH, TERM_ROW_HEIGHT - 4);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_24, 0);
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

void buildTunePage() {
  tunePage = makeBox(screen, PAGE_X, PAGE_Y, PAGE_WIDTH, PAGE_HEIGHT, PANEL, 6);
  makeText(tunePage, &lv_font_montserrat_14, WHITE, "PID TUNE",
           LV_ALIGN_TOP_LEFT, 8, 5);
  makeText(tunePage, &lv_font_montserrat_12, MUTED, "Changes stay in memory",
           LV_ALIGN_TOP_RIGHT, -8, 6);

  testBox =
      makeBox(tunePage, EDIT_X, TEST_ROW_Y, EDIT_WIDTH, TEST_ROW_HEIGHT, TRACK, 4);
  makeText(testBox, &lv_font_montserrat_12, MUTED, "TEST", LV_ALIGN_LEFT_MID,
           12, 0);
  testText = makeText(testBox, &lv_font_montserrat_16, WHITE, "",
                      LV_ALIGN_LEFT_MID, 60, 0);

  for (int term = 0; term < TERM_COUNT; term++) {
    addTermRow(term, TERM_ROW_Y + term * TERM_ROW_GAP);
  }

  resetBox = makeBox(tunePage, EDIT_X, RESET_ROW_Y, EDIT_WIDTH,
                     RESET_ROW_HEIGHT, TRACK, 4);
  makeText(resetBox, &lv_font_montserrat_12, WHITE, "RESET TO CODE VALUES",
           LV_ALIGN_CENTER, 0, 0);

  lv_obj_t *helpBox = makeBox(tunePage, HELP_X, TEST_ROW_Y, HELP_WIDTH,
                              HELP_HEIGHT, BACKGROUND, 4);
  helpText = makeText(helpBox, &lv_font_montserrat_12, MUTED, "",
                      LV_ALIGN_TOP_LEFT, 8, 8);
  lv_obj_set_width(helpText, HELP_WIDTH - 16);
  lv_label_set_long_mode(helpText, LV_LABEL_LONG_WRAP);

  // Built last so it opens on top of the value rows
  dropdownBox = makeBox(tunePage, EDIT_X, TEST_ROW_Y + TEST_ROW_HEIGHT,
                        EDIT_WIDTH, DROPDOWN_ROW_HEIGHT * SYSTEM_COUNT,
                        BACKGROUND, 4);
  for (int system = 0; system < SYSTEM_COUNT; system++) {
    dropdownRows[system] =
        makeBox(dropdownBox, 2, system * DROPDOWN_ROW_HEIGHT + 1,
                EDIT_WIDTH - 4, DROPDOWN_ROW_HEIGHT - 2, TRACK, 3);
    makeText(dropdownRows[system], &lv_font_montserrat_16, WHITE,
             SYSTEM_TITLES[system], LV_ALIGN_LEFT_MID, 12, 0);
  }
  lv_obj_add_flag(dropdownBox, LV_OBJ_FLAG_HIDDEN);
}

void buildScreen() {
  screen = lv_obj_create(nullptr);
  lv_obj_set_style_bg_color(screen, lv_color_hex(BACKGROUND), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
  lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);
  buildGraphPage();
  buildTunePage();
}

void setHidden(lv_obj_t *object, bool hidden) {
  if (hidden) {
    lv_obj_add_flag(object, LV_OBJ_FLAG_HIDDEN);
  } else {
    lv_obj_remove_flag(object, LV_OBJ_FLAG_HIDDEN);
  }
}

void drawPage(const View &v) {
  const int page = v.page.load();
  if (page != shownPage) {
    shownPage = page;
    setHidden(graphPage, page != PAGE_GRAPH);
    setHidden(tunePage, page != PAGE_TUNE);
  }
}

// Tune page

void drawTermValues(const View &v) {
  const int system = v.system.load();
  if (system != shownSystem) {
    shownSystem = system;
    lv_label_set_text(testText, SYSTEM_TITLES[system]);
    for (TermRow &row : termRows) {
      row.shownValue = -1;
    }
  }
  for (int term = 0; term < TERM_COUNT; term++) {
    const int value = v.gains[system][term].load();
    TermRow &row = termRows[term];
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

void drawHelp(int focus) {
  const char *help[] = {
      "Left/Right\nchange test\n\nA: open list\nDown: values\nY: graph",
      "Up/Down\npick a test\n\nA: select\nB: back",
      "Up/Down\npick a value\n\nA: edit\nB: back\nY: graph",
      "Left/Right\npick a digit\n\nUp/Down\nroll 0 to 9\n\nA: done",
  };
  lv_label_set_text(helpText, help[focus]);
}

void drawSelection(const View &v) {
  const int focus = v.focus.load();
  const int valueRow = v.valueRow.load();
  const int digit = v.digit.load();
  const int dropdownRow = v.dropdownRow.load();
  const int key = focus * 1000 + valueRow * 100 + digit * 10 + dropdownRow;
  if (key == shownSelection) {
    return;
  }
  shownSelection = key;

  const bool onValues = focus == FOCUS_VALUES || focus == FOCUS_DIGIT;
  lv_obj_set_style_bg_color(
      testBox, lv_color_hex(focus == FOCUS_TEST ? SELECTED : TRACK), 0);
  for (int term = 0; term < TERM_COUNT; term++) {
    lv_obj_set_style_bg_color(
        termRows[term].box,
        lv_color_hex(onValues && term == valueRow ? SELECTED : TRACK), 0);
    for (int d = 0; d < DIGIT_COUNT; d++) {
      const bool active = focus == FOCUS_DIGIT && term == valueRow && d == digit;
      lv_obj_set_style_bg_opa(termRows[term].digits[d],
                              active ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
      lv_obj_set_style_text_color(termRows[term].digits[d],
                                  lv_color_hex(active ? BACKGROUND : WHITE), 0);
    }
  }
  lv_obj_set_style_bg_color(
      resetBox,
      lv_color_hex(focus == FOCUS_VALUES && valueRow == RESET_ROW ? SELECTED
                                                                  : TRACK),
      0);

  setHidden(dropdownBox, focus != FOCUS_DROPDOWN);
  for (int system = 0; system < SYSTEM_COUNT; system++) {
    lv_obj_set_style_bg_color(
        dropdownRows[system],
        lv_color_hex(system == dropdownRow ? SELECTED : TRACK), 0);
  }
  drawHelp(focus);
}

// Graph page

void drawGainSummary(const View &v) {
  const int system = v.system.load();
  const int kP = v.gains[system][0].load();
  const int kI = v.gains[system][1].load();
  const int kD = v.gains[system][2].load();
  if (system == shownGains[0] && kP == shownGains[1] && kI == shownGains[2] &&
      kD == shownGains[3]) {
    return;
  }
  shownGains[0] = system;
  shownGains[1] = kP;
  shownGains[2] = kI;
  shownGains[3] = kD;
  lv_label_set_text_fmt(gainText, "%s  P %d.%02d  I %d.%02d  D %d.%02d",
                        SYSTEM_NAMES[system], kP / 100, kP % 100, kI / 100,
                        kI % 100, kD / 100, kD % 100);
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
    const char *hints[] = {"A runs the test, Y edits the values",
                           "Overshoots: raise kD or lower kP",
                           "Stops short: raise kP",
                           "Slow to settle: raise kP",
                           "Looks good, copy these into the code",
                           "Lift rotation sensor not found"};
    const std::uint32_t colors[] = {MUTED, YELLOW, YELLOW, YELLOW, GREEN, RED};
    lv_label_set_text(hintText, phase == PHASE_RUNNING
                                    ? "A, B, or a stick stops the test"
                                    : hints[hint]);
    lv_obj_set_style_text_color(
        hintText, lv_color_hex(phase == PHASE_RUNNING ? MUTED : colors[hint]),
        0);
  }

  if (phase == PHASE_READY || v.moveNumber.load() == 0) {
    return;
  }
  const int system = v.testSystem.load();
  const int number = v.moveNumber.load();
  const int size = static_cast<int>(std::lround(v.moveSize.load()));
  const int title = system * 100000 + number * 1000 + size;
  if (title != shownTitle) {
    shownTitle = title;
    lv_label_set_text_fmt(titleText, "%s %d%s  %d/%d", SYSTEM_NAMES[system],
                          size, system == SYS_DRIVE ? " in" : "\xC2\xB0",
                          number, v.moveCount.load());
  }
}

void setLength(lv_obj_t *label, int &shownValue, float value, bool inches,
               const char *prefix) {
  const int tenths = static_cast<int>(std::lround(value * 10));
  if (tenths == shownValue) {
    return;
  }
  shownValue = tenths;
  lv_label_set_text_fmt(label, "%s%d.%d%s", prefix, tenths / 10, tenths % 10,
                        inches ? " in" : "\xC2\xB0");
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
  if (v.phase.load() == PHASE_READY || v.moveNumber.load() == 0) {
    return;
  }
  const bool inches = v.testSystem.load() == SYS_DRIVE;
  setLength(metricText[0], shownMetric[0], v.overshoot.load(), inches, "");
  setLength(metricText[1], shownMetric[1], v.finalError.load(), inches, "");
  setSeconds(metricText[2], shownMetric[2], v.settleTime.load(), "");
  setLength(worstText[0], shownWorst[0], v.worstOvershoot.load(), inches,
            "worst ");
  setLength(worstText[1], shownWorst[1], v.worstError.load(), inches, "worst ");
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
    drawPage(v);
    drawTermValues(v);
    drawSelection(v);
    drawGainSummary(v);
    drawStatus(v);
    drawMetrics(v);
    if (v.phase.load() != PHASE_READY && v.moveNumber.load() != 0) {
      drawGraph(v);
    }
  }
}

} // namespace

void startScreen() { lv_timer_create(refresh, REFRESH_MS, nullptr); }

} // namespace PidTune
