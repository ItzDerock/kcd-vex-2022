#include "auton.hpp"
#include "main.h"
#include "pros/llemu.hpp"

#ifndef AUTON_CPP
#define AUTON_CPP

namespace auton {

AutonType auton_type = NONE;
GameSide game_side = RED;

void run() {
  // switch (auton_type) {
  // case SIDE:
  //   if (game_side == RED) {
  //     run_red_side();
  //   } else {
  //     run_blue_side();
  //   }
  //   break;
  // case BACK:
  //   if (game_side == RED) {
  //     run_red_back();
  //   } else {
  //     run_blue_back();
  //   }
  //   break;
  // case SKILLS:
  //   run_skills();
  //   break;
  // case NONE:
  //   run_none();
  //   break;
  // }
}

void updateDisplay() {
  pros::lcd::set_text(
      2, "Auton: " +
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