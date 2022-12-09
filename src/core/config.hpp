#include "main.h"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include <memory>

#ifndef CONFIG_HPP
#define CONFIG_HPP

/**
 * Drive train ports
 */
#define DRIVE_TOP_LEFT 1
#define DRIVE_TOP_RIGHT 2
#define DRIVE_BOTTOM_LEFT 3
#define DRIVE_BOTTOM_RIGHT 4

#define INERTIAL_PORT 11

/**
 * Catapult ports
 */
#define CATAPULT_MOTOR 5
#define CATAPULT_POTENTIOMETER 1

/**
 * Intake ports
 */
#define INTAKE_MOTOR 6

/**
 * Roller thing port
 */
#define ROLLER_MOTOR 7

/**
 * utility macros
 */
#define BUTTON(x) if (master.get_digital_new_press(x))
#define HELD(x) if (master.get_digital(x))

extern std::shared_ptr<OdomChassisController> chassis;
extern std::shared_ptr<XDriveModel> model;
extern std::shared_ptr<pros::IMU> inertial;
extern std::shared_ptr<Motor> catapult_motor;
extern std::shared_ptr<pros::ADIAnalogIn> catapult_pot;
extern std::shared_ptr<Motor> intake_motor;
extern std::shared_ptr<Motor> roller_motor;

extern bool chassis_break;
extern bool auto_reload;

// catapult states
enum Catapult { REELING, READY_TO_LAUNCH, LAUNCHING, IDLE };
extern Catapult catapult_state;

#endif
// } // namespace bot