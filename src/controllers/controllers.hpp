#ifndef CONTROLLER_CONTROLLER_HPP
#define CONTROLLER_CONTROLLER_HPP

namespace auton {
enum AutonType { RED_SIDE, BLUE_SIDE, RED_BACK, BLUE_BACK, SKILLS, NONE };

void run_red_side();
void run_blue_side();
void run_red_back();
void run_blue_back();

void run_skills();
void run_none();
} // namespace auton

#endif // CONTROLLER_CONTROLLER_HPP