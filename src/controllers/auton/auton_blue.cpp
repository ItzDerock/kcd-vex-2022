#include "../../core/config.hpp"
#include "../movement/movement.hpp"
#include "main.h"

#ifndef AUTON_BLUE_CPP
#define AUTON_BLUE_CPP

namespace auton {

void run_blue_side() {
  // chassis->driveToPoint({0_ft, 1_ft});
  movement::moveTo(0, 50);
}
void run_blue_back() {}

} // namespace auton

#endif // AUTON_RED_CPP
