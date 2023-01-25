#include "pid.hpp"

PIDController::PIDController(double kP, double kI, double kD)
    : _kP(kP), _kI(kI), _kD(kD) {}

double PIDController::update(double error) {
  _integral += error;
  double derivative = error - _previousError;
  double output = _kP * error + _kI * _integral + _kD * derivative;
  _previousError = error;
  return output;
}

void PIDController::reset() {
  _previousError = 0;
  _integral = 0;
}
