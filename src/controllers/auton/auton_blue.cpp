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

  movement::moveTo(12, -12, 45);

  // wait for cata to go down
  movement::loadCatapultSync();

  // enable intake
  intake_motor->moveVelocity(200);

  // move to pick up disks
  movement::setMaxVelocity(0.50);
  // movement::moveTo(37, 20, 45);
  movement::moveTo(43, 27, 45);

  // stop intake
  // intake_motor->moveVelocity(0);
  movement::setMaxVelocity(127);

  // turn to face goal
  movement::setAngleTolerance(1);
  movement::setTolerance(1);
  // movement::moveTo(45, 26, 360 - 42);
  movement::moveTo(45, 26, 90 + 42);

  pros::delay(1000);
  intake_motor->moveVelocity(-200);
  pros::delay(1000);

  // fire disks
  intake_motor->moveVelocity(0);
  movement::fireCatapultSync();

  // load and fire again
  // movement::loadCatapultSync();
  // pros::delay(250);
  // intake_motor->moveVelocity(200);
  // pros::delay(2000);
  // intake_motor->moveVelocity(0);
  // movement::fireCatapultSync();

  // second roller
  movement::setAngleTolerance(2);
  movement::setTolerance(2);
  movement::moveTo(49, 49, 90);
  movement::moveTo(56, 65, 90);
  movement::moveTo(90, 74, 90);

  // do roller
  model->xArcade(-75, 0, 0);
  pros::delay(500);
  roller_motor->moveRelative(40, 50);
  model->xArcade(0, 0, 0);

  // done
  movement::setChassisBreak(false);
  inertial->tare_heading();
}

} // namespace auton

#endif // AUTON_RED_CPP
