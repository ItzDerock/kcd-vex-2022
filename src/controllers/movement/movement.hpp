#ifndef MOVEMENT_HPP
#define MOVEMENT_HPP

#include "okapi/api/units/QLength.hpp"
namespace movement {

extern double x;
extern double y;
extern double z;

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
