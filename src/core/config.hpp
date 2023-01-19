#include "main.h"
#include "okapi/api/chassis/controller/odomChassisController.hpp"
#include "okapi/impl/device/rotarysensor/adiEncoder.hpp"
#include "pros/adi.hpp"
#include "pros/rtos.hpp"
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

#define INERTIAL_PORT 10

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
#define ROLLER_MOTOR 20

/**
 * END GAME
 */
#define END_GAME_PNEUMATIC 2

/**
 * Odometry
 */
#define ODOM_RIGHT_1 'C'
#define ODOM_RIGHT_2 'D'

#define ODOM_LEFT_1 'G'
#define ODOM_LEFT_2 'H'

#define ODOM_MIDDLE_1 'E'
#define ODOM_MIDDLE_2 'F'

/**
 * utility macros
 */
#define BUTTON(x) if (master.get_digital_new_press(x))
#define HELD(x) if (master.get_digital(x))

extern std::shared_ptr<ChassisController> chassis;
extern std::shared_ptr<XDriveModel> model;
extern std::shared_ptr<pros::IMU> inertial;
extern std::shared_ptr<Motor> catapult_motor;
extern std::shared_ptr<pros::ADIAnalogIn> catapult_pot;
extern std::shared_ptr<Motor> intake_motor;
extern std::shared_ptr<Motor> roller_motor;
extern std::shared_ptr<pros::ADIDigitalOut> endgame_launcher;
extern std::shared_ptr<ADIEncoder> left;
extern std::shared_ptr<ADIEncoder> right;
extern std::shared_ptr<ADIEncoder> middle;

extern bool chassis_break;
extern bool auto_reload;

// catapult states
enum Catapult { REELING, READY_TO_LAUNCH, LAUNCHING, IDLE, DISABLED };
extern Catapult catapult_state;

#endif
// } // namespace bot
