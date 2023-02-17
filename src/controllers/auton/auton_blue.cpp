#include "../../core/config.hpp"
#include "../movement/movement.hpp"
#include "./auton_partials.hpp"
#include "main.h"

#ifndef AUTON_BLUE_CPP
#define AUTON_BLUE_CPP

namespace auton {

void run_blue_side() {
  movement::setChassisBreak(true);
  movement::setAngleTolerance(1);
  movement::moveTo(0, 0, 370 - 37);
  // chassis->driveToPoint({0_ft, 1_ft});
}
void run_blue_back() {
  movement::setChassisBreak(true);

  // move to back roller;
  backFarRoller();

  movement::moveTo(3, -9, 45);

  // wait for cata to go down
  movement::loadCatapultSync();

  // enable intake
  intake_motor->moveVelocity(200);

  // move to pick up disks
  movement::setMaxVelocity(0.50);
  // movement::moveTo(24, 13, 45);
  movement::moveTo(22, 9, 45);

  // stop intake
  // pros::delay(250);
  intake_motor->moveVelocity(0);
  movement::setMaxVelocity(127);

  // turn to face goal
  movement::setAngleTolerance(1);
  movement::setTolerance(1);
  movement::moveTo(34, 23, 360 - 37);
  // pros::delay(500);
  // movement::turnTo(360 - 30, 1);

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
