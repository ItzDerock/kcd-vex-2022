#pragma once

class PIDController {
public:
  PIDController(double kP, double kI, double kD);
  double update(double error);
  void reset();

private:
  double _kP, _kI, _kD;
  double _previousError = 0;
  double _integral = 0;
};
