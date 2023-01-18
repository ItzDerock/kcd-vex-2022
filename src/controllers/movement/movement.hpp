#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

#include "okapi/api/units/QLength.hpp"

namespace movement {

class Point {
public:
  Point(double x, double y, double angle) : x(x), y(y), angle(angle){};
  Point(double x, double y) : x(x), y(y), angle(0){};
  double x;
  double y;
  double angle;
};

extern Point previousPosition;
extern Point position;

void updatePosition();
void updatePositionLoop();
void resetPosition();

void moveDistance(okapi::QLength distance);
void moveDistance(okapi::QLength distance, int maxVelocity);
void turnAngle(double angle);
void moveDistanceAsync(double distance);
void turnAngleAsync(double angle);
void waitUntilSettled();
void stop();
} // namespace movement

#endif // MOVEMENT_HPP
