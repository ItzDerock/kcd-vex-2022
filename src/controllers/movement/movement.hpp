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

extern Point goalLocation;

/**
 * Move to a location (absolute).
 * Max velocity between 1 and 127.
 */
void moveTo(double x, double y, double finalAngle, double maxVelocity,
            double maxError);
void moveTo(double x, double y, double finalAngle, double maxVelocity);
void moveTo(double x, double y, double finalAngle);
void moveTo(double x, double y);

/**
 * Turn to an angle (absolute).
 */
void turnTo(double angle);

} // namespace movement
#endif // MOVEMENT_HPP
