// #pragma once

#include "movement.hpp"
#include "../../core/config.hpp"
#include "../../util.hpp"
#include "main.h"
#include "odom.hpp"
#include "pid.hpp"
#include <cmath>

double MINIMUM_ERROR = 0.05;

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

Point goalLocation = Point(0, 0, 0);

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

PIDController xPID = PIDController(0.1, 0.0001, 0.001);
PIDController yPID = PIDController(0.1, 0.0001, 0.001);
PIDController anglePID = PIDController(0.1, 0.0001, 0.001);

void moveTo(double x, double y, double targetAngle, int maxVelocity) {
  // reset the PID systems
  xPID.reset();
  yPID.reset();
  anglePID.reset();

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
    // double requiredAngle = fmin(std::fabs(targetAngle - currHeading),
    //                             360 - std::fabs(targetAngle - currHeading));
    double requiredAngle =
        180 - std::fabs(std::fmod(std::abs(targetAngle - currHeading), 360.0) -
                        180.0);

    // Calculate the PID values
    double xPower = xPID.update(requiredX);
    double yPower = yPID.update(requiredY);
    double anglePower = anglePID.update(requiredAngle);

    // Break loop if we are close enough
    if (fabs(requiredX) < MINIMUM_ERROR && fabs(requiredY) < MINIMUM_ERROR &&
        fabs(requiredAngle) < 2)
      break;

    // log for debugging
    printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n", odom::globalPoint.x,
           odom::globalPoint.y, currHeading, requiredX, requiredY,
           requiredAngle, xPower, yPower, anglePower);

    // convert to field centered
    std::pair<double, double> fieldCentered = toFieldCentered(xPower, yPower);

    // set to zero if error is too small
    if (fabs(requiredX) < MINIMUM_ERROR)
      fieldCentered.first = 0;

    if (fabs(requiredY) < MINIMUM_ERROR)
      fieldCentered.second = 0;

    // if (fabs(requiredAngle) < 2)
    anglePower = 0;

    // Set the motor power
    model->xArcade(
        utils::constrain(fieldCentered.first, -maxVelocity, maxVelocity),
        utils::constrain(fieldCentered.second, -maxVelocity, maxVelocity),
        utils::constrain(anglePower, -maxVelocity, maxVelocity));

    pros::delay(10);
  }

  // Stop the motors
  model->stop();
}

// overloads
void moveTo(double x, double y, double targetAngle) {
  moveTo(x, y, targetAngle, 127);
}

void moveTo(double x, double y) { moveTo(x, y, odom::globalPoint.angle, 127); }

// turn to
void turnTo(double angle) {
  // Reset the PID system
  anglePID.reset();

  while (true) {
    // Calculate dt
    // double dt = pros::millis() - lastTime;

    // read sensor
    double currHeading = inertial->get_heading();

    // Calculate distance and angle to target
    double requiredAngle = fmin(std::fabs(angle - currHeading),
                                360 - std::fabs(angle - currHeading));

    // Calculate the PID values
    double anglePower = anglePID.update(requiredAngle);

    // Break loop if we are close enough
    if (fabs(requiredAngle) < 2)
      break;

    // Set the motor power
    model->xArcade(0, 0, anglePower);

    pros::delay(10);
  }
}

} // namespace movement
