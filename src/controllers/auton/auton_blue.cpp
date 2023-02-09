#include "../../core/config.hpp"
#include "../movement/movement.hpp"
#include "main.h"

#ifndef AUTON_BLUE_CPP
#define AUTON_BLUE_CPP

namespace auton {

void run_blue_side() {
  // chassis->driveToPoint({0_ft, 1_ft});
}
void run_blue_back() {
  movement::setChassisBreak(true);
  movement::moveTo(0, -24, 0);

  pros::delay(500);
  model->xArcade(-20, 0, 0);
  pros::delay(500);
  roller_motor->moveRelative(120, 50);
  model->xArcade(0, 0, 0);

  pros::delay(250);
  movement::moveTo(5, -15, 45);

  // wait for cata to go down
  catapult_motor->moveVelocity(65);
  while (catapult_pot->get_value() < CATAPULT_POT_LOADING) {
    pros::delay(10);
  }
  catapult_motor->moveVelocity(0);

  // enable intake
  intake_motor->moveVelocity(200);

  // move to pick up disks
  movement::setMaxVelocity(75);
  movement::moveTo(42, 24, 45);

  // stop intake
  pros::delay(1000);
  intake_motor->moveVelocity(0);
  movement::setMaxVelocity(127);

  // turn to face goal
  movement::turnTo(360 - 35);

  // fire disks
  catapult_motor->moveVelocity(100);
  while (catapult_pot->get_value() > CATAPULT_POT_LAUNCHED) {
    pros::delay(10);
  }
  catapult_motor->moveVelocity(0);
}

} // namespace auton

#endif // AUTON_RED_CPP
