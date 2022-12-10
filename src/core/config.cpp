#include "config.hpp"
#include "main.h"
#include <memory>

#ifndef CONFIG_CPP
#define CONFIG_CPP

/**
 * define inertial sensor
 */
std::shared_ptr<pros::IMU> inertial =
    std::make_shared<pros::IMU>(INERTIAL_PORT);

/**
 * define chassis and model
 */
std::shared_ptr<OdomChassisController> chassis =
    ChassisControllerBuilder()
        .withMotors(DRIVE_TOP_LEFT, -DRIVE_TOP_RIGHT, -DRIVE_BOTTOM_RIGHT,
                    DRIVE_BOTTOM_LEFT)
        .withDimensions(AbstractMotor::gearset::green,
                        {{4_in, 12_in}, imev5GreenTPR})
        .withGains({0.001, 0, 0.0001}, {0.001, 0, 0.0001}, {0.001, 0, 0.0001})
        .withOdometry(StateMode::FRAME_TRANSFORMATION)
        .buildOdometry();

std::shared_ptr<XDriveModel> model =
    std::static_pointer_cast<XDriveModel>(chassis->getModel());

/**
 * define catapult motor and potentiometer
 */
std::shared_ptr<Motor> catapult_motor = std::make_shared<okapi::Motor>(
    CATAPULT_MOTOR, true, AbstractMotor::gearset::red,
    AbstractMotor::encoderUnits::degrees);

std::shared_ptr<pros::ADIAnalogIn> catapult_pot =
    std::make_shared<pros::ADIAnalogIn>(CATAPULT_POTENTIOMETER);

/**
 * define intake motor
 */
std::shared_ptr<Motor> intake_motor = std::make_shared<okapi::Motor>(
    INTAKE_MOTOR, true, AbstractMotor::gearset::green,
    AbstractMotor::encoderUnits::degrees);

/**
 * define roller motor
 */
std::shared_ptr<Motor> roller_motor = std::make_shared<okapi::Motor>(
    ROLLER_MOTOR, true, AbstractMotor::gearset::red,
    AbstractMotor::encoderUnits::degrees);

/**
 * Define endgame
 */
std::shared_ptr<pros::ADIDigitalOut> endgame_launcher =
    std::make_shared<pros::ADIDigitalOut>(END_GAME_PNEUMATIC);

/**
 * states
 */
bool chassis_break = false;
bool auto_reload = true;

/**
 * Mutexes
 */
pros::Mutex inertial_mutex;

// catapult states
Catapult catapult_state = IDLE;

#endif