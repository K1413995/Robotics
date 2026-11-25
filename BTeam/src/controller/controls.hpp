#pragma once

#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cstdint>
#include <cstdlib>

class Stick {
private:
  pros::controller_analog_e_t channel;
  int current = 0;

public:
  explicit Stick(pros::controller_analog_e_t analog) : channel(analog) {}

  void update();

  int value() const { return current; } // -127 to 127
  bool past(int amount) const { return std::abs(current) > amount; }
};

class Button {
private:
  pros::controller_digital_e_t control;
  std::uint32_t holdThreshold;

  std::uint32_t startedHolding = 0;

  bool active = false;
  bool pressedEvent = false;
  bool releasedEvent = false;
  bool tapEvent = false;

public:
  Button(pros::controller_digital_e_t digital,
         std::uint32_t holdThreshold = 150)
      : control(digital), holdThreshold(holdThreshold) {}

  void update();

  bool pressed() const { return active; }
  bool newPress() const { return pressedEvent; }
  bool released() const { return releasedEvent; }
  bool held() const;
  bool tapped() const { return tapEvent; }
  pros::controller_digital_e_t id() const { return control; }
};

struct Controls {
  Stick leftY{pros::E_CONTROLLER_ANALOG_LEFT_Y};
  Stick rightX{pros::E_CONTROLLER_ANALOG_RIGHT_X};
  Stick rightY{pros::E_CONTROLLER_ANALOG_RIGHT_Y};

  Button liftUp{pros::E_CONTROLLER_DIGITAL_L1};
  Button liftDown{pros::E_CONTROLLER_DIGITAL_L2};
  Button toggleMacro{pros::E_CONTROLLER_DIGITAL_X};
  Button turnLeft{pros::E_CONTROLLER_DIGITAL_LEFT};
  Button turnRight{pros::E_CONTROLLER_DIGITAL_RIGHT};

  // On the same button, a tap opens and a hold (150 ms) runs the intake
  Button clawIntake{pros::E_CONTROLLER_DIGITAL_R2};
  Button clawOpen{pros::E_CONTROLLER_DIGITAL_R2};

  // Menus and the PID tuner
  Button menuUp{pros::E_CONTROLLER_DIGITAL_UP};
  Button menuDown{pros::E_CONTROLLER_DIGITAL_DOWN};
  Button menuLeft{pros::E_CONTROLLER_DIGITAL_LEFT};
  Button menuRight{pros::E_CONTROLLER_DIGITAL_RIGHT};
  Button menuSelect{pros::E_CONTROLLER_DIGITAL_A};
  Button menuBack{pros::E_CONTROLLER_DIGITAL_B};

  // Test code only: double tap to switch modes
  Button autoMode{pros::E_CONTROLLER_DIGITAL_A};
  Button driverMode{pros::E_CONTROLLER_DIGITAL_X};
  Button pidMode{pros::E_CONTROLLER_DIGITAL_Y};

  static void update(Controls &driver);
};

// Driver profiles

inline Controls base{};
inline Controls john{.liftUp = Button(pros::E_CONTROLLER_DIGITAL_B),
                     .liftDown = Button(pros::E_CONTROLLER_DIGITAL_DOWN),
                     .toggleMacro = Button(pros::E_CONTROLLER_DIGITAL_L2),
                     .clawOpen = Button(pros::E_CONTROLLER_DIGITAL_R1)};
// Like john, but the claw is all on R2 (tap opens, hold intakes)
// and the cardinal turns are on L1 (left) and R1 (right)
inline Controls nikolai{.liftUp = Button(pros::E_CONTROLLER_DIGITAL_B),
                        .liftDown = Button(pros::E_CONTROLLER_DIGITAL_DOWN),
                        .toggleMacro = Button(pros::E_CONTROLLER_DIGITAL_L2),
                        .turnLeft = Button(pros::E_CONTROLLER_DIGITAL_L1),
                        .turnRight = Button(pros::E_CONTROLLER_DIGITAL_R1)};
