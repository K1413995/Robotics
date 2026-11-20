#pragma once

#include "pros/misc.h"
#include "pros/rtos.hpp"
#include <cstdint>

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
    Button(pros::controller_digital_e_t digital, std::uint32_t holdThreshold = 300)
        : control(digital), holdThreshold(holdThreshold) {}

    void update();

    bool pressed() const {return active;}
    bool newPress() const {return pressedEvent;}
    bool released() const {return releasedEvent;}
    bool held() const;
    bool tapped() const {return tapEvent;}
};


struct Controls {
    Button auton{pros::E_CONTROLLER_DIGITAL_A};

    pros::controller_analog_e_t leftY = pros::E_CONTROLLER_ANALOG_LEFT_Y;
    pros::controller_analog_e_t rightX = pros::E_CONTROLLER_ANALOG_RIGHT_X;

    Button liftUp{pros::E_CONTROLLER_DIGITAL_L1};
    Button liftDown{pros::E_CONTROLLER_DIGITAL_L2};

    Button clawIntake{pros::E_CONTROLLER_DIGITAL_R2};
    Button clawOpen{pros::E_CONTROLLER_DIGITAL_R2};

    Button toggle{pros::E_CONTROLLER_DIGITAL_Y};

    static void update(Controls& driver);
};


// Driver profiles

inline Controls base{};
inline Controls john{
    .liftUp = Button(pros::E_CONTROLLER_DIGITAL_B),
    .liftDown = Button(pros::E_CONTROLLER_DIGITAL_DOWN),
    .clawOpen = Button(pros::E_CONTROLLER_DIGITAL_R1),
    .toggle = Button(pros::E_CONTROLLER_DIGITAL_L2)
};