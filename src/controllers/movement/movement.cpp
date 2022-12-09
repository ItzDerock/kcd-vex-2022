// #pragma once

#include "../../core/config.hpp"
// #include "../../util.hpp"
#include "main.h"
#include "pid.hpp"

double wrap_degrees(double angle) { return std::fmod(angle + 180, 360) - 180; }

namespace movement {

void moveDistance(okapi::QLength distance, int maxVelPercent) {
  // create the bounds for motor speeds in tank drive
  double maxSpeed = maxVelPercent / 100.0 * 127;

  // Create a PID controller with appropriate constants for IMU drift correction
  PIDController pidController(0.5, 0.5, 0.3, -(maxVelPercent / 2),
                              (maxVelPercent / 2));

  // Set the target distance and initial error
  double targetDistance = distance.convert(meter);
  double error = targetDistance;

  // Set the initial IMU angle and previous error for the PID controller
  double initialAngle = inertial->get_heading();
  pidController.previousError = 0;

  // Set the initial time for calculating the derivative term in the PID
  // controller
  double initialTime = pros::millis();

  double currentAngle = inertial->get_heading();

  int i = 0;
  // Loop until the error is within a certain tolerance
  while (i < 1000) {
    i++;
    // Calculate the current time and elapsed time since the last iteration
    double currentTime = pros::millis();
    double dt = (currentTime - initialTime) / 1000.0;

    // Get the current IMU angle and calculate the error
    currentAngle = inertial->get_heading();
    error = wrap_degrees(currentAngle - initialAngle);

    printf("Current Angle: %f\n", currentAngle);
    printf("Initial Angle: %f\n", initialAngle);
    printf("Error: %f\n", error);

    // Calculate the correction output from the PID controller
    double correction = pidController.calculate(error, dt);

    // Calculate the left and right velocities
    double left = maxVelPercent - correction;
    double right = maxVelPercent + correction;

    printf("Left: %f, Right: %f\n", left, right);
    printf("Correction: %f\n\n", correction);

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

void moveDistance(QLength distance) { moveDistance(distance, 600); }

} // namespace movement
