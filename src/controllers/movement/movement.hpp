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

void moveTo(double x, double y, double finalAngle);

} // namespace movement
#endif // MOVEMENT_HPP
