#pragma once

#include "movement.hpp"
#include "../../core/config.hpp"
#include "../../util.hpp"
#include "main.h"
#include "odom.hpp"
#include "pid.hpp"
#include <cmath>

double kP = 12;
double kD = 7.5;
double turn_kP = 175;
double turn_kD = 100;

double wrap_degrees(double angle) { return std::fmod(angle + 180, 360) - 180; }

namespace movement {

void setRawVelocity(double LF, double LB, double RF, double RB) {
  auto blMotor = model->getBottomLeftMotor();
  auto brMotor = model->getBottomRightMotor();
  auto tlMotor = model->getTopLeftMotor();
  auto trMotor = model->getTopRightMotor();

  // set the values
  blMotor->moveVelocity(LB);
  brMotor->moveVelocity(RB);
  tlMotor->moveVelocity(LF);
  trMotor->moveVelocity(RF);

  // print velocities
  printf("LF: %f, LB: %f, RF: %f, RB: %f\n", LF, LB, RF, RB);
}

void moveTo__legacy(double x, double y, double finalAngle) {
  double startAngle = odom::globalPoint.angle;

  // some PID variables
  double derivative_x = 0.0;
  double derivative_y = 0.0;
  double prevError_x = 0.0;
  double prevError_y = 0.0;
  double turn_prevError = 0.0;

  while (true) {
    // Calculates distance and angle of the current point to the target point
    double distanceToTarget =
        hypot(x - odom::globalPoint.x, y - odom::globalPoint.y);
    // dist = distanceToTarget; // for multitasking
    double angleToTarget =
        atan2(y - odom::globalPoint.y, x - odom::globalPoint.x);
    double relativeAngleToTarget =
        utils::angleWrap(angleToTarget + odom::globalPoint.angle); // - ?

    // Calculates the x and y components of the vector
    double relativeXToTarget = (cos(relativeAngleToTarget) * distanceToTarget);
    double relativeYToTarget = (sin(relativeAngleToTarget) * distanceToTarget);

    // Movement PID
    derivative_x = relativeAngleToTarget - prevError_x;
    prevError_x = relativeAngleToTarget;
    derivative_y = relativeAngleToTarget - prevError_y;
    prevError_y = relativeAngleToTarget;

    double movementXPower = relativeXToTarget * kP + derivative_x * kD;
    double movementYPower = relativeYToTarget * kP + derivative_x * kD;

    // Calculates turn angle
    double heading2 =
        finalAngle < 0 ? finalAngle + M_PI * 2 : finalAngle - M_PI * 2;
    finalAngle = (fabs(odom::globalPoint.angle) - finalAngle <
                  fabs(odom::globalPoint.angle) - heading2)
                     ? finalAngle
                     : heading2;
    printf("turn amount: %f\n", heading2);

    // Turn PID
    double turnError = -(
        finalAngle - utils::compressAngle(startAngle, odom::globalPoint.angle));
    // turn_e = turnError;
    int turn_derivative = turnError - turn_prevError;
    int turn_prevError = turnError;
    double turnPower = turnError * turn_kP + turn_derivative * turn_kD;

    printf("turn error: %f\n", turnError);

    if (fabs(turnError) < utils::getRadians(2) &&
        fabs(relativeXToTarget) < 0.5 && fabs(relativeYToTarget) < 0.5)
      break;

    printf("moving");
    setRawVelocity(movementYPower + movementXPower - turnPower,
                   movementYPower - movementXPower - turnPower,
                   movementYPower - movementXPower + turnPower,
                   movementYPower + movementXPower + turnPower);

    // wait(10, msec);
    pros::delay(10);
  }

  printf("exited pid");
  setRawVelocity(0, 0, 0, 0);
}

auto toFieldCentered(double rightSpeed, double forwardSpeed) {
  // get heading
  double heading = inertial->get_heading();

  // convert input to radians
  heading *= (M_PI / 180);

  // calculate the new rightSpeed and forwardSpeed
  double temp = rightSpeed * cos(heading) - forwardSpeed * sin(heading);
  forwardSpeed = rightSpeed * sin(heading) + forwardSpeed * cos(heading);
  rightSpeed = temp;

  return std::make_pair(rightSpeed, forwardSpeed);
}

void moveTo(double x, double y, double targetAngle, int maxVelocity) {
  // Create three PID systems,
  // one for each axis and one for the angle

  PIDController xPID =
      PIDController(0.5, 0.5, 0.3, -(maxVelocity), maxVelocity);
  PIDController yPID =
      PIDController(0.5, 0.5, 0.3, -(maxVelocity), maxVelocity);
  PIDController anglePID =
      PIDController(0.5, 0.5, 0.3, -maxVelocity, maxVelocity);

  // track the time before the last run
  double lastTime = pros::millis();

  // debug csv header
  printf("x, y, angle, reqX, reqY, reqAng, xPow, yPow, angPow\n");

  // Run the loop
  while (true) {
    // Calculate dt
    double dt = pros::millis() - lastTime;

    // read sensor
    double currHeading = inertial->get_heading();

    // Calculate distance and angle to target
    double requiredX = x - odom::globalPoint.x;
    double requiredY = y - odom::globalPoint.y;
    // double requiredAngle = wrap_degrees(currHeading - targetAngle);
    double requiredAngle = fmin(std::fabs(targetAngle - currHeading),
                                360 - std::fabs(targetAngle - currHeading));

    // Calculate the PID values
    double xPower = xPID.calculate(requiredX, dt);
    double yPower = yPID.calculate(requiredY, dt);
    double anglePower = anglePID.calculate(requiredAngle, dt);

    // Break loop if we are close enough
    if (fabs(requiredX) < 0.5 && fabs(requiredY) < 0.5 &&
        fabs(requiredAngle) < 2)
      break;

    // log for debugging
    printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n", odom::globalPoint.x,
           odom::globalPoint.y, currHeading, requiredX, requiredY,
           requiredAngle, xPower, yPower, anglePower);

    // convert to field centered
    std::pair<double, double> fieldCentered = toFieldCentered(xPower, yPower);

    // Set the motor power
    model->xArcade(fieldCentered.first, fieldCentered.second, anglePower);

    pros::delay(10);
  }

  // Stop the motors
  model->stop();
}

// overloads
void moveTo(double x, double y, double targetAngle) {
  moveTo(x, y, targetAngle, 127);
}

} // namespace movement
