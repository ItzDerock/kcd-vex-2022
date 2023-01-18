// Big thanks to jazonshou/4253B-Offseason-X-Drive
// i would be clueless without that

#include "./odom.hpp"
#include "../../core/config.hpp"
#include "../../util.hpp"
#include "main.h"
#include "movement.hpp"

// constants
// #define BACK_ENC_OFFSET = 5.75;
double BACK_ENC_OFFSET = 5.75;

// GLOBAL COORDINATES
movement::Point odom::globalPoint = {0, 0, 0};
movement::Point odom::prevGlobalPoint = {0, 0, 0};
movement::Point odom::globalDeltaPoint = {0, 0, 0};

// LOCAL COORDINATES
movement::Point odom::localDeltaPoint = {0, 0};

// SENSOR VALUES
// encoder
int leftEnc = 0;
int rightEnc = 0;
int backEnc = 0;

int prevLeftEnc = 0;
int prevRightEnc = 0;
int prevBackEnc = 0;

int deltaLeftEnc = 0;
int deltaRightEnc = 0;
int deltaBackEnc = 0;

// angle
double currentAngle = 0.0;
double prevAngle = 0.0;
double deltaAngle = 0.0;

// Odometry Functions
void odom::updateSensors() {
  leftEnc = utils::degToInch(left->get());
  rightEnc = utils::degToInch(right->get());
  backEnc = utils::degToInch(-middle->get());

  // find deltas
  deltaLeftEnc = leftEnc - prevLeftEnc;
  deltaRightEnc = rightEnc - prevRightEnc;
  deltaBackEnc = backEnc - prevBackEnc;

  // set prev values
  prevLeftEnc = leftEnc;
  prevRightEnc = rightEnc;
  prevBackEnc = rightEnc;

  // get current angle
  currentAngle = utils::getRadians(inertial->get_rotation());
  deltaAngle = currentAngle - prevAngle;
  prevAngle = currentAngle;
}

// Update position
void odom::updatePosition() {
  // Polar coordinates
  localDeltaPoint.x =
      (deltaAngle + (deltaBackEnc / BACK_ENC_OFFSET)) * BACK_ENC_OFFSET;
  localDeltaPoint.y = (deltaLeftEnc + deltaRightEnc) / 2.0;

  // Cartesian coordinates
  globalDeltaPoint.x = (localDeltaPoint.y * sin(prevAngle + deltaAngle / 2)) +
                       (localDeltaPoint.x * cos(prevAngle + deltaAngle / 2));
  globalDeltaPoint.y = (localDeltaPoint.y * cos(prevAngle + deltaAngle / 2)) -
                       (localDeltaPoint.x * sin(prevAngle + deltaAngle / 2));
  globalDeltaPoint.angle = deltaAngle;

  // X & y Position
  globalPoint.x = globalDeltaPoint.x + prevGlobalPoint.x;
  globalPoint.y = globalDeltaPoint.y + prevGlobalPoint.y;
  globalPoint.angle = currentAngle;

  prevGlobalPoint.x = globalPoint.x;
  prevGlobalPoint.y = globalPoint.y;
}

// reset
void odom::reset() {
  left->reset();
  right->reset();
  middle->reset();
  prevLeftEnc = 0.0;
  prevRightEnc = 0.0;
  prevBackEnc = 0.0;
  prevAngle = 0.0;
  prevGlobalPoint.x = 0.0;
  prevGlobalPoint.y = 0.0;
}

void odom::setPosition(double newX, double newY, double newAngle) {
  reset();
  prevAngle = newAngle;
  prevGlobalPoint.x = newX;
  prevGlobalPoint.y = newY;
}

// ODOMETRY THREAD
void odom::run() {
  while (true) {
    odom::updateSensors();
    odom::updatePosition();

    pros::delay(10);
  }
}
