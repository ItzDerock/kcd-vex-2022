#include "auton.hpp"
#include "../../core/config.hpp"
#include "../movement/movement.hpp"
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

  movement::moveTo(100, 100, 90);

  // model->xArcade(-50, 0, 0);
  // pros::delay(500);
  // model->xArcade(0, 0, 0);
  //
  // if (game_side == BLUE) {
  //   roller_motor->moveRelative(-150, 100);
  // } else {
  //   roller_motor->moveRelative(150, 100);
  // }
  //
  // move backwards slowly and spin roller
  // chassis->setMaxVelocity(SLOW);
  // chassis->moveDistanceAsync(-5_in);

  // chassis->stop();
  // chassis->setMaxVelocity(600);

  // chassis->moveDistance(120_in);
  // x-drive move at 45 deg angle for 120 inches
  // chassis->driveToPoint({120_in, 120_in});
  // movement::turnAngle(90);

  // chassis->setMaxVelocity(SLOW);
  // chassis->moveDistanceAsync(-5_in);
  // roller_motor->moveRelative(10, 100);
  // chassis->setMaxVelocity(600);

  // movement::moveDistance(47_in, 100);
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
