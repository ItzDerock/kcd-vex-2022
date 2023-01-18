#include "../../core/config.hpp"
#include "main.h"
#include "movement.hpp"
#include "pros/llemu.hpp"
#include <string>

namespace movement {

movement::Point previousPosition(0, 0);
movement::Point position(0, 0);
double previous_time = pros::millis() / 1000.0;

FILE *usd_write_file = fopen("/usd/accel.txt", "w");

// update using inertial (pros::IMU)
void updatePosition() {
  double curr_time = pros::millis() / 1000.0;
  double dt = curr_time - previous_time;

  // check if calibrating
  if (inertial->is_calibrating()) {
    printf("imucal\n");
    pros::delay(1000);
    return;
  }

  // get accel
  auto accel = inertial->get_accel();

  if (errno != 0) {
    printf("Error (i): %s (%d)", strerror(errno), errno);
    return;
  }

  // print the output
  // printf("%f,%f", accel.x, accel.y);
  fputs((std::to_string(dt) + ", " + std::to_string(accel.x) + ", " +
         std::to_string(accel.y) + "\n")
            .c_str(),
        usd_write_file);

  if (errno != 0) {
    printf("Error (sd): %s (%d)", strerror(errno), errno);
  }

  // if accel is under 0.05 then it is probably noise
  if (std::abs(accel.x) < 0.05) {
    accel.x = 0;
  }
  if (std::abs(accel.y) < 0.05) {
    accel.y = 0;
  }

  // Calculate the change in position based on the acceleration values and the
  // time elapsed
  double dx = (accel.x + previousPosition.x) * dt / 2;
  double dy = (accel.y + previousPosition.y) * dt / 2;

  position.x += dx;
  position.y += dy;

  // update previous
  previousPosition.x = accel.x;
  previousPosition.y = accel.y;

  pros::lcd::set_text(2, "X: " + std::to_string(position.x) +
                             " Y: " + std::to_string(position.y));

  // update previous time
  previous_time = pros::millis() / 1000.0;
  return;
}

void updatePositionLoop() {
  // fclose(usd_write_file);
  while (true) {
    updatePosition();
    pros::Task::delay(10);
  }
}

void cleanup() {
  // do cleanup
}

void resetPosition() {
  fclose(usd_write_file);
  // position.x = 0;
  // position.y = 0;
}

} // namespace movement
