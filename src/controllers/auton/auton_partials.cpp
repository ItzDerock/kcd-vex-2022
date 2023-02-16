#include "../../core/config.hpp"
#include "../movement/movement.hpp"

void backFarRoller() {
  movement::moveTo(0, -17, 0);

  model->xArcade(-20, 0, 0);
  pros::delay(500);
  roller_motor->moveRelative(120, 50);
  model->xArcade(0, 0, 0);
}
