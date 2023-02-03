#ifndef AUTON_CONTROLLER_HPP
#define AUTON_CONTROLLER_HPP

namespace auton {

enum GameSide { RED, BLUE };
enum AutonType { SIDE, BACK, BACK_SHORT, SKILLS, NONE };

extern AutonType auton_type;
extern GameSide game_side;

void run_red_side();
void run_blue_side();
void run_red_back(bool quickMode);
void run_blue_back();

void run_skills();
void run_none();

void run();
void toggleSide();
void toggleMode();
void updateDisplay();

} // namespace auton

#endif // AUTON_CONTROLLER_HPP
