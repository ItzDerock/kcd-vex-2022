#include "../../core/config.hpp"
#include "../movement/movement.hpp"
#include "../movement/odom.hpp"
#include "main.h"

#ifndef AUTON_RED_CPP
#define AUTON_RED_CPP

namespace auton {

void run_red_back(bool quickMode) {
  movement::setChassisBreak(true);
  movement::moveTo(0, -24, 0);

  pros::delay(500);
  model->xArcade(-20, 0, 0);
  pros::delay(500);
  roller_motor->moveRelative(150, 50);
  model->xArcade(0, 0, 0);

  // done with first roller
  pros::delay(500);

  if (!quickMode) {
    movement::moveTo(0, 0, -20);

    // center area
    movement::setAngleTolerance(10);
    movement::moveTo(45, 27, -20, 127, 5);
    pros::delay(100);
    movement::moveTo(52, 64, 0, 127, 5);
    movement::setAngleTolerance(2);

    movement::moveTo(90, 64, 90, 127, 1.25);
    pros::delay(200);
    model->xArcade(-20, 0, 0);
    pros::delay(1000);
    roller_motor->moveRelative(130, 50);
    pros::delay(100);
    model->xArcade(0, 0, 0);
  } else {
    movement::turnTo(90);
  }

  pros::delay(500);
  movement::setChassisBreak(false);

  // tare
  inertial->tare_heading();
}
void run_red_side() {}

void run_skills() {}
void run_none() {}

} // namespace auton

#endif // AUTON_RED_CPP
