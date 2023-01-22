#pragma once
#include "pid.hpp"
#include "okapi/api.hpp"

PIDController::PIDController(double kp, double ki, double kd, double minOutput,
                             double maxOutput)
    : kp(kp), ki(ki), kd(kd), minOutput(minOutput), maxOutput(maxOutput) {}

double PIDController::calculate(double error, double dt) {
  integral += error * dt;
  double derivative = (error - previousError) / dt;
  double output = kp * error + ki * integral + kd * derivative;
  previousError = error;

  if (output > maxOutput) {
    output = maxOutput;
  } else if (output < minOutput) {
    output = minOutput;
  }

  return output;
}

double PIDController::calculate(double error) { return calculate(error, 1); }

void PIDController::reset() {
  integral = 0;
  previousError = 0;
}

void PIDController::setMinMaxOutput(double minOutput, double maxOutput) {
  this->minOutput = minOutput;
  this->maxOutput = maxOutput;
}
