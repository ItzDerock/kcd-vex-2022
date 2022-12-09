// #pragma once

#include "../../core/config.hpp"
// #include "../../util.hpp"
#include "main.h"
#include "pid.hpp"

double wrap_degrees(double angle) { return std::fmod(angle + 180, 360) - 180; }

namespace movement {

void moveDistance(okapi::QLength distance) {
  // Create a PID controller with appropriate constants for IMU drift correction
  PIDController pidController(0.5, 0.5, 0.3, -0.5, 0.5);

  // Set the target distance and initial error
  double targetDistance = distance.convert(meter);
  double error = targetDistance;

  // Set the initial IMU angle and previous error for the PID controller
  double initialAngle = inertial->get_heading() + 180;
  // if over 360 degrees, subtract 360
  if (initialAngle > 360) {
    initialAngle -= 360;
  }

  pidController.previousError = initialAngle;

  // Set the initial time for calculating the derivative term in the PID
  // controller
  double initialTime = pros::millis();

  double currentAngle = inertial->get_heading();

  // Loop until the error is within a certain tolerance
  while (std::abs(error) > 0.1) {
    // Calculate the current time and elapsed time since the last iteration
    double currentTime = pros::millis();
    double dt = (currentTime - initialTime) / 1000.0;

    // Get the current IMU angle and calculate the error
    currentAngle = inertial->get_heading();
    error = targetDistance - wrap_degrees(currentAngle - initialAngle);

    // Calculate the correction output from the PID controller
    double correction = pidController.calculate(error, dt);

    // Set the left and right side speeds with the correction applied
    double left =
        std::abs(targetDistance / distance.convert(meter)) - correction;
    double right =
        std::abs(targetDistance / distance.convert(meter)) + correction;

    // Set the chassis speeds
    model->tank(left, right);

    // Update the initial time for the next iteration
    initialTime = currentTime;

    // Delay the loop to prevent hogging the CPU
    pros::delay(10);
  }

  // Stop the chassis once the target distance is reached
  chassis->stop();
}

} // namespace movement
