#include "../../core/config.hpp"
#include "main.h"
#include "movement.hpp"
#include "pros/llemu.hpp"

namespace movement {

double x = 0;
double y = 0;
double z = 0;

// update using inertial (pros::IMU)
void updatePosition(int lastUpdate) {
  // inertial_mutex.take();
  auto accel = inertial->get_accel();
  // inertial_mutex.give();

  if (errno != 0) {
    printf("Error: %s (%d)", strerror(errno), errno);
    return;
  }

  // turn acceleration (in Gs) into distance traveled
  // x += accel.x *

  x += accel.x;
  y += accel.y;
  z += accel.z;

  pros::lcd::set_text(2, "X: " + std::to_string(x) + " Y: " +
                             std::to_string(y) + " Z: " + std::to_string(z));
}

void updatePositionLoop() {
  while (true) {
    updatePosition();
    pros::Task::delay(10);
  }
}

void resetPosition() {
  x = 0;
  y = 0;
  z = 0;
}

} // namespace movement