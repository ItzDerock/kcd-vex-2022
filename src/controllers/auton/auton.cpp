#include "auton.hpp"
#include "../../core/config.hpp"
#include "main.h"
#include "pros/llemu.hpp"

#ifndef AUTON_CPP
#define AUTON_CPP

#define SLOW 300

namespace auton {

AutonType auton_type = NONE;
GameSide game_side = RED;

void run() {
  if (auton_type == NONE) {
    printf("no auton");
    return;
  }

  auton_chassis->setTarget({10_in, 0_in, 0_deg}, true);

  // movement::moveTo(25, 36, 0);
}

void updateDisplay() {
  pros::lcd::set_text(
      3, "Auton: " +
             std::string(auton_type == SIDE     ? "SIDE"
                         : auton_type == BACK   ? "BACK"
                         : auton_type == SKILLS ? "SKILLS"
                                                : "NONE") +
             " | Side: " + std::string(game_side == RED ? "RED" : "BLUE"));
}

void toggleMode() {
  switch (auton_type) {
  case SIDE:
    auton_type = BACK;
    break;
  case BACK:
    auton_type = SKILLS;
    break;
  case SKILLS:
    auton_type = NONE;
    break;
  case NONE:
    auton_type = SIDE;
    break;
  }

  updateDisplay();
}

void toggleSide() {
  switch (game_side) {
  case RED:
    game_side = BLUE;
    break;
  case BLUE:
    game_side = RED;
    break;
  }

  updateDisplay();
}

} // namespace auton

#endif // AUTON_CPP
