#pragma once
#include "main.h"
#include "movement.hpp"

namespace odom {

extern movement::Point globalPoint;
extern movement::Point prevGlobalPoint;
extern movement::Point globalDeltaPoint;

extern movement::Point localDeltaPoint;

// functions

void updateSensors();
void updatePosition();
void reset();
void setPosition(double newX, double newY, double newAngle);
void run();

} // namespace odom
