#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

#include "okapi/api/units/QLength.hpp"
namespace movement {
void moveDistance(okapi::QLength distance);
void turnAngle(double angle);
void moveDistanceAsync(double distance);
void turnAngleAsync(double angle);
void waitUntilSettled();
void stop();
} // namespace movement

#endif // MOVEMENT_HPP
