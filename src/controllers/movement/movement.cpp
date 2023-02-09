// #pragma once

#include "movement.hpp"
#include "../../core/config.hpp"
#include "../../util.hpp"
#include "main.h"
#include "odom.hpp"
#include "pid.hpp"
#include <cmath>

double MINIMUM_ERROR = 2;
double ANGLE_ERROR_TOLERANCE = 2;

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

// PIDController xPID = PIDController(0.2, 0.00005, 0.002);
// PIDController yPID = PIDController(0.2, 0.00005, 0.002);
// PIDController xPID(1.5, 0.00002, 0.001);
// PIDController yPID(1.5, 0.00002, 0.001);
// 0.25 = good
PIDController xPID(0.125, 0.00005, 0.002);
PIDController yPID(0.125, 0.00005, 0.002);
PIDController anglePID = PIDController(0.02, 0.0001, 0.001);

void setAngleTolerance(double tolerance) { ANGLE_ERROR_TOLERANCE = tolerance; }

void moveTo(double x, double y, double targetAngle, double maxVelocity) {
  // reset the PID systems
  xPID.reset();
  yPID.reset();
  anglePID.reset();

  // track the time before the last run
  double lastTime = pros::millis();

  // debug csv header
  // printf("x, y, angle, reqX, reqY, reqAng, xPow, yPow, angPow\n");

  // Run the loop
  while (true) {
    // Calculate dt
    double dt = pros::millis() - lastTime;

    // read sensor
    double currHeading = inertial->get_heading();

    // Calculate distance and angle to target
    double requiredX = x - odom::globalPoint.x;
    double requiredY = y - odom::globalPoint.y;
    double requiredAngle = targetAngle - currHeading;

    // wrap
    if (requiredAngle > 180)
      requiredAngle = -1 * (360 + requiredAngle);
    else if (requiredAngle < -180)
      requiredAngle = 360 + requiredAngle;

    // if targetAngle is -1, don't turn
    if (targetAngle == -1)
      requiredAngle = 0;

    // Calculate the PID values
    double xPower = xPID.update(requiredX);
    double yPower = yPID.update(requiredY);
    double anglePower = anglePID.update(requiredAngle);

    // Break loop if we are close enough
    if (fabs(requiredX) < MINIMUM_ERROR && fabs(requiredY) < MINIMUM_ERROR &&
        fabs(requiredAngle) < ANGLE_ERROR_TOLERANCE)
      break;

    // log for debugging
    // printf("%f, %f, %f, %f, %f, %f, %f, %f, %f\n", odom::globalPoint.x,
    //        odom::globalPoint.y, currHeading, requiredX, requiredY,
    //        requiredAngle, xPower, yPower, anglePower);

    // convert to field centered
    std::pair<double, double> fieldCentered = toFieldCentered(xPower, yPower);

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

void moveTo(double x, double y, double targetAngle, double maxVelocity,
            double maxError) {
  double oldError = MINIMUM_ERROR;
  MINIMUM_ERROR = maxError;
  moveTo(x, y, targetAngle, maxVelocity);
  MINIMUM_ERROR = oldError;
}

// turn to
void turnTo(double targetAngle) {
  // Reset the PID system
  anglePID.reset();

  while (true) {
    // Calculate dt

    // read sensor
    double currHeading = inertial->get_heading();

    // Calculate distance and angle to target
    double requiredAngle = targetAngle - currHeading;

    // wrap
    if (requiredAngle > 180)
      requiredAngle = -1 * (360 + requiredAngle);
    else if (requiredAngle < -180)
      requiredAngle = 360 + requiredAngle;

    // Calculate the PID values
    double anglePower = anglePID.update(requiredAngle);

    // Break loop if we are close enough
    if (fabs(requiredAngle) < 2)
      break;

    // Set the motor power
    model->xArcade(0, 0, anglePower);

    pros::delay(10);
  }

  model->stop();
}

bool chassis_break = false;
void setChassisBreak(bool value) {
  chassis_break = value;
  model->setBrakeMode(chassis_break ? AbstractMotor::brakeMode::hold
                                    : AbstractMotor::brakeMode::coast);

  // doesn't work, set brake mode for each motor individually
  auto blMotor = model->getBottomLeftMotor();
  auto brMotor = model->getBottomRightMotor();
  auto tlMotor = model->getTopLeftMotor();
  auto trMotor = model->getTopRightMotor();

  for (auto motor : {blMotor, brMotor, tlMotor, trMotor}) {
    motor->setBrakeMode(chassis_break ? AbstractMotor::brakeMode::hold
                                      : AbstractMotor::brakeMode::coast);
  }

  pros::lcd::set_text(4, "Chassis Break: " + std::to_string(chassis_break));
}

void toggleChassisBreak() { setChassisBreak(!chassis_break); }

} // namespace movement
