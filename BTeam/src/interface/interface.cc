// INTERFACE
// Brain screen with position, motor temps, distance sensors, and lift rotation
#include "interface.hpp"
#include "../components/components.hpp"
#include "../hardware/hardware.hpp"
#include "liblvgl/lvgl.h"

#include <atomic>
#include <cmath>
#include <cstdint>
#include <cstdlib>

namespace Interface {
namespace {

// Set from the driver task, drawn on the display task
std::atomic<const char *> modeLabel{nullptr};
const char *shownMode = nullptr;
lv_obj_t *teamText = nullptr;

// SETTINGS
// Motor turns yellow at this temperature, in Celsius
constexpr int WARM_TEMP = 45;
// Motor turns red at this temperature, in Celsius
constexpr int HOT_TEMP = 55;
// How often the screen updates, in milliseconds
constexpr int REFRESH_MS = 200;
// true shows lengths in centimeters like Jerry.io, false shows inches
constexpr bool POSITION_IN_CM = true;

// COLORS
constexpr std::uint32_t BACKGROUND = 0x0B1220;
constexpr std::uint32_t HEADER = 0x3432C7;
constexpr std::uint32_t PANEL = 0x182235;
constexpr std::uint32_t TRACK = 0x2B3750;
constexpr std::uint32_t WHITE = 0xFFFFFF;
constexpr std::uint32_t MUTED = 0x9AA6BA;
constexpr std::uint32_t GREEN = 0x34D399;
constexpr std::uint32_t YELLOW = 0xFBBF24;
constexpr std::uint32_t RED = 0xF87171;
constexpr std::uint32_t BLUE = 0x60A5FA;

// LAYOUT, the brain's drawing area is 480 x 240
constexpr int SCREEN_WIDTH = 480;
constexpr int HEADER_HEIGHT = 26;
constexpr int POSITION_X[] = {78, 182, 286};
constexpr int PANEL_Y = 30;
constexpr int LEFT_PANEL_X = 6;
constexpr int LEFT_PANEL_WIDTH = 296;
constexpr int MOTOR_PANEL_HEIGHT = 112;
constexpr int MOTOR_COLUMN_WIDTH = 148;
constexpr int MOTOR_ROW_Y = 20;
constexpr int MOTOR_ROW_HEIGHT = 22;
constexpr int DISTANCE_PANEL_Y = 146;
constexpr int DISTANCE_PANEL_HEIGHT = 86;
constexpr int DISTANCE_CELL_WIDTH = 96;
constexpr int ROTATION_X = 308;
constexpr int ROTATION_WIDTH = 166;
constexpr int ROTATION_HEIGHT = 202;
constexpr int ARC_SIZE = 116;

constexpr int MOTOR_COUNT = 7;
constexpr int DISTANCE_COUNT = 3;
constexpr int NOT_DRAWN = -2;
constexpr int MISSING = -1;
constexpr int NO_WALL = -3;
// The distance sensor returns this when it can't see anything
constexpr int NO_OBJECT_MM = 9999;
// Position can be negative, so it needs its own "no value" markers
constexpr int POSITION_NOT_DRAWN = INT32_MIN;
constexpr int POSITION_MISSING = INT32_MAX;

struct MotorTile {
  pros::AbstractMotor *motor;
  std::uint8_t index;
  lv_obj_t *temp;
  lv_obj_t *marker;
  int shown;
};

struct DistanceCell {
  pros::Distance *sensor;
  int port;
  lv_obj_t *value;
  lv_obj_t *detail;
  int shown;
};

MotorTile tiles[MOTOR_COUNT];
DistanceCell distanceCells[DISTANCE_COUNT];
lv_obj_t *statusText = nullptr;
lv_obj_t *rotationArc = nullptr;
lv_obj_t *rotationValue = nullptr;
lv_obj_t *rotationStatus = nullptr;
int shownStatus = NOT_DRAWN;
int shownRotation = NOT_DRAWN;
lv_obj_t *positionText[3] = {nullptr, nullptr, nullptr};
int shownPosition[3] = {POSITION_NOT_DRAWN, POSITION_NOT_DRAWN,
                        POSITION_NOT_DRAWN};

const char *lengthUnit() { return POSITION_IN_CM ? " cm" : " in"; }

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

void addTile(lv_obj_t *panel, int slot, const char *name,
             pros::AbstractMotor &motor, std::uint8_t index, int port,
             int column, int row) {
  const int x = column * MOTOR_COLUMN_WIDTH;
  const int y = MOTOR_ROW_Y + row * MOTOR_ROW_HEIGHT;

  lv_obj_t *marker = makeBox(panel, x + 8, y + 3, 3, 14, TRACK, 1);
  makeText(panel, &lv_font_montserrat_12, WHITE, name, LV_ALIGN_TOP_LEFT,
           x + 15, y + 3);
  lv_obj_t *portText = makeText(panel, &lv_font_montserrat_10, MUTED, "",
                                LV_ALIGN_TOP_LEFT, x + 66, y + 5);
  lv_label_set_text_fmt(portText, "P%d", port);
  lv_obj_t *temp =
      makeText(panel, &lv_font_montserrat_14, MUTED, "--", LV_ALIGN_TOP_RIGHT,
               x + MOTOR_COLUMN_WIDTH - 8 - LEFT_PANEL_WIDTH, y + 2);

  tiles[slot] = MotorTile{&motor, index, temp, marker, NOT_DRAWN};
}

void addDistanceCell(lv_obj_t *panel, int slot, const char *name,
                     pros::Distance &sensor, int port) {
  const int x = (slot - 1) * DISTANCE_CELL_WIDTH;
  makeText(panel, &lv_font_montserrat_12, MUTED, name, LV_ALIGN_TOP_MID, x, 20);
  lv_obj_t *value = makeText(panel, &lv_font_montserrat_16, MUTED, "--",
                             LV_ALIGN_TOP_MID, x, 37);
  lv_obj_t *detail = makeText(panel, &lv_font_montserrat_10, MUTED, "",
                              LV_ALIGN_TOP_MID, x, 62);
  lv_label_set_text_fmt(detail, "Port %d", port);
  distanceCells[slot] = DistanceCell{&sensor, port, value, detail, NOT_DRAWN};
}

std::uint32_t tempColor(int temp, bool overTemp) {
  if (overTemp || temp >= HOT_TEMP) {
    return RED;
  }
  return temp >= WARM_TEMP ? YELLOW : GREEN;
}

void refreshRotation() {
  const std::int32_t angle = Hardware::liftRotation.get_angle();
  const bool found = angle != PROS_ERR;
  const int tenths = found ? angle / 10 : MISSING;
  if (tenths == shownRotation) {
    return;
  }
  shownRotation = tenths;

  if (found) {
    lv_label_set_text_fmt(rotationValue, "%d.%d\xC2\xB0", tenths / 10,
                          tenths % 10);
    lv_obj_set_style_text_color(rotationValue, lv_color_hex(WHITE), 0);
    lv_arc_set_value(rotationArc, tenths);
    lv_label_set_text(rotationStatus, "CONNECTED");
    lv_obj_set_style_text_color(rotationStatus, lv_color_hex(GREEN), 0);
  } else {
    lv_label_set_text(rotationValue, "--");
    lv_obj_set_style_text_color(rotationValue, lv_color_hex(MUTED), 0);
    lv_arc_set_value(rotationArc, 0);
    lv_label_set_text(rotationStatus, "NOT FOUND");
    lv_obj_set_style_text_color(rotationStatus, lv_color_hex(RED), 0);
  }
}

void refreshDistances() {
  for (DistanceCell &cell : distanceCells) {
    const std::int32_t mm = cell.sensor->get_distance();
    const int value =
        mm == PROS_ERR ? MISSING : (mm >= NO_OBJECT_MM ? NO_WALL : mm);
    if (value == cell.shown) {
      continue;
    }

    // Only change the small text when the sensor connects or disconnects
    if (value == MISSING) {
      lv_label_set_text(cell.detail, "NOT FOUND");
      lv_obj_set_style_text_color(cell.detail, lv_color_hex(RED), 0);
    } else if (cell.shown == MISSING) {
      lv_label_set_text_fmt(cell.detail, "Port %d", cell.port);
      lv_obj_set_style_text_color(cell.detail, lv_color_hex(MUTED), 0);
    }
    cell.shown = value;

    if (value == MISSING) {
      lv_label_set_text(cell.value, "--");
      lv_obj_set_style_text_color(cell.value, lv_color_hex(MUTED), 0);
    } else if (value == NO_WALL) {
      lv_label_set_text(cell.value, "FAR");
      lv_obj_set_style_text_color(cell.value, lv_color_hex(MUTED), 0);
    } else {
      const int tenths =
          POSITION_IN_CM ? value : static_cast<int>(std::lround(value / 2.54));
      lv_label_set_text_fmt(cell.value, "%d.%d%s", tenths / 10, tenths % 10,
                            lengthUnit());
      lv_obj_set_style_text_color(cell.value, lv_color_hex(WHITE), 0);
    }
  }
}

// Tenths let the label show one decimal without printing a float
int toTenths(double value) {
  return std::isfinite(value) ? static_cast<int>(std::lround(value * 10))
                              : POSITION_MISSING;
}

void showPositionValue(int slot, char name, int tenths, const char *unit) {
  if (tenths == shownPosition[slot]) {
    return;
  }
  shownPosition[slot] = tenths;

  if (tenths == POSITION_MISSING) {
    lv_label_set_text_fmt(positionText[slot], "%c --", name);
    return;
  }
  const int size = std::abs(tenths);
  lv_label_set_text_fmt(positionText[slot], "%c %s%d.%d%s", name,
                        tenths < 0 ? "-" : "", size / 10, size % 10, unit);
}

void refreshPosition() {
  const lemlib::Pose pose = chassis.getPose();
  const double scale = POSITION_IN_CM ? 2.54 : 1.0;
  showPositionValue(0, 'X', toTenths(pose.x * scale), lengthUnit());
  showPositionValue(1, 'Y', toTenths(pose.y * scale), lengthUnit());

  // Keep heading between 0 and 359.9, even after several full turns
  int heading =
      toTenths(std::fmod(std::fmod(pose.theta, 360.0) + 360.0, 360.0));
  if (heading != POSITION_MISSING) {
    heading %= 3600;
  }
  showPositionValue(2, 'H', heading, "\xC2\xB0");
}

void refresh(lv_timer_t *) {
  int hot = 0;
  int missing = 0;

  for (MotorTile &tile : tiles) {
    const double reading = tile.motor->get_temperature(tile.index);
    const bool found = std::isfinite(reading);
    const int tenths = found ? static_cast<int>(std::lround(reading * 10)) : 0;
    const int temp = tenths / 10;
    const bool overTemp = found && tile.motor->is_over_temp(tile.index) == 1;

    if (!found) {
      missing++;
    } else if (overTemp || temp >= HOT_TEMP) {
      hot++;
    }

    // Only redraw a motor when its value changes
    const int value = found ? tenths * 2 + (overTemp ? 1 : 0) : MISSING;
    if (value == tile.shown) {
      continue;
    }
    tile.shown = value;

    if (found) {
      const lv_color_t color = lv_color_hex(tempColor(temp, overTemp));
      // Shows the motor's exact reading, with a decimal if it sends one
      if (tenths % 10 == 0) {
        lv_label_set_text_fmt(tile.temp,
                              "%d\xC2\xB0"
                              "C",
                              temp);
      } else {
        lv_label_set_text_fmt(tile.temp,
                              "%d.%d\xC2\xB0"
                              "C",
                              temp, tenths % 10);
      }
      lv_obj_set_style_text_color(tile.temp, color, 0);
      lv_obj_set_style_bg_color(tile.marker, color, 0);
    } else {
      lv_label_set_text(tile.temp, "--");
      lv_obj_set_style_text_color(tile.temp, lv_color_hex(MUTED), 0);
      lv_obj_set_style_bg_color(tile.marker, lv_color_hex(TRACK), 0);
    }
  }

  // Header shows the most important problem
  const int status = hot > 0 ? 100 + hot : missing;
  if (status != shownStatus) {
    shownStatus = status;
    if (hot > 0) {
      lv_label_set_text_fmt(statusText, "%d HOT", hot);
      lv_obj_set_style_text_color(statusText, lv_color_hex(RED), 0);
    } else if (missing > 0) {
      lv_label_set_text_fmt(statusText, "%d MISSING", missing);
      lv_obj_set_style_text_color(statusText, lv_color_hex(YELLOW), 0);
    } else {
      lv_label_set_text(statusText, "ALL GOOD");
      lv_obj_set_style_text_color(statusText, lv_color_hex(GREEN), 0);
    }
  }

  const char *label = modeLabel.load();
  if (label != nullptr && label != shownMode) {
    shownMode = label;
    lv_label_set_text(teamText, label);
  }

  refreshDistances();
  refreshRotation();
  refreshPosition();
}

void buildMotorPanel(lv_obj_t *screen) {
  lv_obj_t *panel = makeBox(screen, LEFT_PANEL_X, PANEL_Y, LEFT_PANEL_WIDTH,
                            MOTOR_PANEL_HEIGHT, PANEL, 6);
  makeText(panel, &lv_font_montserrat_10, MUTED, "DRIVE", LV_ALIGN_TOP_LEFT, 8,
           5);
  makeText(panel, &lv_font_montserrat_10, MUTED, "MECHANISMS",
           LV_ALIGN_TOP_LEFT, MOTOR_COLUMN_WIDTH + 8, 5);

  addTile(panel, 0, "L Front", Hardware::leftMotors, 0,
          std::abs(Ports::LEFT_FRONT), 0, 0);
  addTile(panel, 1, "L Back", Hardware::leftMotors, 1,
          std::abs(Ports::LEFT_BACK), 0, 1);
  addTile(panel, 2, "R Front", Hardware::rightMotors, 0,
          std::abs(Ports::RIGHT_FRONT), 0, 2);
  addTile(panel, 3, "R Back", Hardware::rightMotors, 1,
          std::abs(Ports::RIGHT_BACK), 0, 3);
  addTile(panel, 4, "Lift L", Hardware::liftMotors, 0,
          std::abs(Ports::LIFT_LEFT), 1, 0);
  addTile(panel, 5, "Lift R", Hardware::liftMotors, 1,
          std::abs(Ports::LIFT_RIGHT), 1, 1);
  addTile(panel, 6, "Claw", Hardware::claw, 0, std::abs(Ports::CLAW), 1, 2);
}

void buildDistancePanel(lv_obj_t *screen) {
  lv_obj_t *panel = makeBox(screen, LEFT_PANEL_X, DISTANCE_PANEL_Y,
                            LEFT_PANEL_WIDTH, DISTANCE_PANEL_HEIGHT, PANEL, 6);
  makeText(panel, &lv_font_montserrat_10, MUTED, "DISTANCE", LV_ALIGN_TOP_LEFT,
           8, 5);
  addDistanceCell(panel, 0, "Left", Hardware::leftDistance,
                  std::abs(Ports::LEFT_DISTANCE));
  addDistanceCell(panel, 1, "Back", Hardware::backDistance,
                  std::abs(Ports::BACK_DISTANCE));
  addDistanceCell(panel, 2, "Right", Hardware::rightDistance,
                  std::abs(Ports::RIGHT_DISTANCE));
}

void buildRotationPanel(lv_obj_t *screen) {
  lv_obj_t *panel = makeBox(screen, ROTATION_X, PANEL_Y, ROTATION_WIDTH,
                            ROTATION_HEIGHT, PANEL, 6);
  makeText(panel, &lv_font_montserrat_12, MUTED, "LIFT ROTATION",
           LV_ALIGN_TOP_MID, 0, 7);

  rotationArc = lv_arc_create(panel);
  lv_obj_set_size(rotationArc, ARC_SIZE, ARC_SIZE);
  lv_obj_align(rotationArc, LV_ALIGN_TOP_MID, 0, 28);
  lv_arc_set_rotation(rotationArc, 270);
  lv_arc_set_bg_angles(rotationArc, 0, 360);
  lv_arc_set_range(rotationArc, 0, 3600);
  lv_arc_set_value(rotationArc, 0);
  lv_obj_remove_style(rotationArc, nullptr, LV_PART_KNOB);
  lv_obj_remove_flag(rotationArc, LV_OBJ_FLAG_CLICKABLE);
  lv_obj_set_style_arc_width(rotationArc, 10, LV_PART_MAIN);
  lv_obj_set_style_arc_color(rotationArc, lv_color_hex(TRACK), LV_PART_MAIN);
  lv_obj_set_style_arc_width(rotationArc, 10, LV_PART_INDICATOR);
  lv_obj_set_style_arc_color(rotationArc, lv_color_hex(BLUE),
                             LV_PART_INDICATOR);

  rotationValue = makeText(rotationArc, &lv_font_montserrat_24, MUTED, "--",
                           LV_ALIGN_CENTER, 0, 0);

  lv_obj_t *portText = makeText(panel, &lv_font_montserrat_12, MUTED, "",
                                LV_ALIGN_TOP_MID, 0, 152);
  lv_label_set_text_fmt(portText, "Port %d", std::abs(Ports::LIFT_ROTATION));
  rotationStatus = makeText(panel, &lv_font_montserrat_14, MUTED, "--",
                            LV_ALIGN_TOP_MID, 0, 172);
}

void buildScreen(lv_timer_t *) {
  lv_obj_t *screen = lv_screen_active();
  lv_obj_clean(screen);
  lv_obj_set_style_bg_color(screen, lv_color_hex(BACKGROUND), 0);
  lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
  lv_obj_remove_flag(screen, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_t *header =
      makeBox(screen, 0, 0, SCREEN_WIDTH, HEADER_HEIGHT, HEADER, 0);
  teamText = makeText(header, &lv_font_montserrat_14, WHITE, "63310B",
                      LV_ALIGN_LEFT_MID, 8, 0);
  for (int i = 0; i < 3; i++) {
    positionText[i] = makeText(header, &lv_font_montserrat_12, WHITE, "--",
                               LV_ALIGN_LEFT_MID, POSITION_X[i], 0);
  }
  lv_obj_t *statusPill =
      makeBox(header, SCREEN_WIDTH - 92, 3, 86, 20, BACKGROUND, 10);
  statusText = makeText(statusPill, &lv_font_montserrat_12, MUTED, "--",
                        LV_ALIGN_CENTER, 0, 0);

  buildMotorPanel(screen);
  buildDistancePanel(screen);
  buildRotationPanel(screen);

  refresh(nullptr);
  lv_timer_create(refresh, REFRESH_MS, nullptr);
}

} // namespace

void start() {
  // Build on the display's own task so it can't collide with its drawing
  lv_timer_t *timer = lv_timer_create(buildScreen, 0, nullptr);
  lv_timer_set_repeat_count(timer, 1);
}

void setMode(const char *label) { modeLabel = label; }

} // namespace Interface
