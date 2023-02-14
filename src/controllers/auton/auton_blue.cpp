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

  // pros::delay(500);
  model->xArcade(-20, 0, 0);
  pros::delay(500);
  roller_motor->moveRelative(120, 50);
  model->xArcade(0, 0, 0);

  movement::moveTo(5, -15, 45);

  // wait for cata to go down
  movement::loadCatapultSync();

  // enable intake
  intake_motor->moveVelocity(200);

  // move to pick up disks
  movement::setMaxVelocity(0.50);
  movement::moveTo(22, 3, 45);

  // stop intake
  pros::delay(1000);
  intake_motor->moveVelocity(0);
  movement::setMaxVelocity(127);

  // turn to face goal
  movement::turnTo(360 - 35);

  // fire disks
  movement::fireCatapultSync();

  // load and fire again
  movement::loadCatapultSync();
  pros::delay(250);
  intake_motor->moveVelocity(200);
  pros::delay(2000);
  intake_motor->moveVelocity(0);
  movement::fireCatapultSync();

  // done with disks
}

} // namespace auton

#endif // AUTON_RED_CPP
