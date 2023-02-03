#include "../../core/config.hpp"
#include "../movement/movement.hpp"
#include "../movement/odom.hpp"
#include "main.h"

#ifndef AUTON_RED_CPP
#define AUTON_RED_CPP

namespace auton {

void run_red_back() {
  // movement::moveTo(0, -24, 0);

  // while (odom::globalPoint.y > -20) {
  //   model->xArcade(0, -20, 0);
  //   pros::delay(10);
  // }
  // model->xArcade(0, 0, 0);
  model->setBrakeMode(AbstractMotor::brakeMode::hold);

  movement::moveTo(0, -24, 0);

  pros::delay(500);
  model->xArcade(-20, 0, 0);
  pros::delay(500);
  roller_motor->moveRelative(150, 50);
  model->xArcade(0, 0, 0);

  // done with first roller
  pros::delay(500);

  movement::moveTo(0, 0, 0);

  // center area
  movement::moveTo(36, 25, 90, 127, 5);

  movement::moveTo(92, 65, 90);
  pros::delay(50);
  model->xArcade(-20, 0, 0);
  pros::delay(1000);
  roller_motor->moveRelative(130, 50);
  pros::delay(100);
  model->xArcade(0, 0, 0);

  // tare
  inertial->tare_heading();
}
void run_red_side() {}

void run_skills() {}
void run_none() {}

} // namespace auton

#endif // AUTON_RED_CPP
