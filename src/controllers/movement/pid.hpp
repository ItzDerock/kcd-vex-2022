#pragma once

class PIDController {
public:
  PIDController(double kp, double ki, double kd, double minOutput,
                double maxOutput);
  double calculate(double error, double dt);

  // private:
  double kp;
  double ki;
  double kd;
  double minOutput;
  double maxOutput;

  double integral = 0;
  double previousError = 0;
};
