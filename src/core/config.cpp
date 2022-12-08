#include "config.hpp"
#include "main.h"
#include <memory>

#ifndef CONFIG_CPP
#define CONFIG_CPP

/**
 * define inertial sensor
 */
std::shared_ptr<IMU> inertial = std::make_shared<IMU>(INERTIAL_PORT);

/**
 * define chassis and model
 */
std::shared_ptr<OdomChassisController> chassis =
    ChassisControllerBuilder()
        .withMotors(DRIVE_TOP_LEFT, -DRIVE_TOP_RIGHT, -DRIVE_BOTTOM_RIGHT,
                    DRIVE_BOTTOM_LEFT)
        .withDimensions(AbstractMotor::gearset::green,
                        {{4_in, 12.5_in}, imev5GreenTPR})
        .withOdometry()
        .buildOdometry();
//  .build();

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
 * states
 */
bool chassis_break = false;
bool auto_reload = true;

// catapult states
Catapult catapult_state = IDLE;

#endif