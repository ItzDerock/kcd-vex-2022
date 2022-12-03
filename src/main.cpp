#include "main.h"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "util.hpp"
#include <sys/_stdint.h>

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
 * utility macros
 */
#define BUTTON(x) if (master.get_digital_new_press(x))
#define HELD(x) if (master.get_digital(x))

/**
 * define inertial sensor
 */
auto inertial = std::make_shared<IMU>(INERTIAL_PORT);

/**
 * define chassis and model
 */
auto chassis = ChassisControllerBuilder()
                   .withMotors(DRIVE_TOP_LEFT, -DRIVE_TOP_RIGHT,
                               -DRIVE_BOTTOM_RIGHT, DRIVE_BOTTOM_LEFT)
                   .withDimensions(AbstractMotor::gearset::green,
                                   {{4_in, 12.5_in}, imev5GreenTPR})
                   .withOdometry()
                   .buildOdometry();
//  .build();

auto model = std::static_pointer_cast<XDriveModel>(chassis->getModel());

/**
 * define catapult motor and potentiometer
 */
auto catapult_motor = std::make_shared<okapi::Motor>(
    CATAPULT_MOTOR, true, AbstractMotor::gearset::red,
    AbstractMotor::encoderUnits::degrees);

auto catapult_pot = std::make_shared<pros::ADIAnalogIn>(CATAPULT_POTENTIOMETER);

/**
 * define intake motor
 */
auto intake_motor = std::make_shared<okapi::Motor>(
    INTAKE_MOTOR, true, AbstractMotor::gearset::blue,
    AbstractMotor::encoderUnits::degrees);

/**
 * states
 */
bool chassis_break = false;

// catapult states
enum Catapult { REELING, READY_TO_LAUNCH, LAUNCHING, IDLE };
Catapult catapult_state = IDLE;

void run_catapult() {
  // only run if catapult status is "REELING"
  if (catapult_state == REELING) {
    // move until 1200 on potentiometer
    if (catapult_pot->get_value() < 1200) {
      if (catapult_motor->getTargetVelocity() == 0) {
        catapult_motor->moveVelocity(100);
      }
    } else {
      catapult_motor->moveVelocity(0);
      catapult_state = READY_TO_LAUNCH;
    }
  } else

    // only run if catapult status is "LAUNCHING"
    if (catapult_state == LAUNCHING) {
      // move until 15 on potentiometer
      if (catapult_pot->get_value() > 15) {
        if (catapult_motor->getTargetVelocity() == 0) {
          catapult_motor->moveVelocity(75);
        }
      } else {
        catapult_motor->moveVelocity(0);
        catapult_state = IDLE;
      }
    }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "[i] Calibrating IMU and POT...");

  // calibrate IMU
  inertial->calibrate();

  while (inertial->isCalibrating()) {
    pros::delay(10);
  }

  catapult_motor->setBrakeMode(AbstractMotor::brakeMode::hold);

  // calibrate catapult potentiometer
  catapult_pot->calibrate();

  pros::lcd::set_text(1, "[i] Ready to rumble!");

  // flywheel->setGearing(AbstractMotor::gearset::blue);
  // flywheel->setBrakeMode(AbstractMotor::brakeMode::coast);
  // flywheel->setEncoderUnits(AbstractMotor::encoderUnits::degrees);
  // flywheel->tarePosition();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() { pros::lcd::set_text(1, "[i] Robot Disabled"); }

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  pros::lcd::set_text(1, "[i] Autonomous Started");
  chassis->moveDistance(12_in);
  pros::lcd::set_text(1, "[i] Autonomous Ended");
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  while (true) {
    // run catapult updater
    run_catapult();

    // get joystick values
    double irightSpeed = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    double iforwardSpeed = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double irotSpeed = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // scale joystick values
    double rightSpeed = utils::mapValue(irightSpeed, -127, 127, -1, 1);
    double forwardSpeed = utils::mapValue(iforwardSpeed, -127, 127, -1, 1);
    double rotationSpeed = utils::mapValue(irotSpeed, -127, 127, -1, 1);

    // deadzone, if all values are below 0.1 return
    if (fabs(rightSpeed) < 0.1 && fabs(forwardSpeed) < 0.1 &&
        fabs(rotationSpeed) < 0.1) {
      pros::lcd::set_text(2, "Chassis: Stopped");
      chassis->stop();
    } else {
      // set chassis speed
      model->xArcade(rightSpeed, forwardSpeed, rotationSpeed);

      // print right, forward, rot on 2nd line of LCD
      pros::lcd::set_text(2, "R: " + utils::round(rightSpeed, 2) +
                                 " Fw: " + utils::round(forwardSpeed, 2) +
                                 " Rt: " + utils::round(rotationSpeed, 2));
    }

    // toggle chassis break
    BUTTON(pros::E_CONTROLLER_DIGITAL_L2) {
      chassis_break = !chassis_break;
      model->setBrakeMode(chassis_break ? AbstractMotor::brakeMode::hold
                                        : AbstractMotor::brakeMode::coast);

      // doesn't work, set brake mode for each motor individually
      auto blMotor = model->getBottomLeftMotor();
      auto brMotor = model->getBottomRightMotor();
      auto tlMotor = model->getTopLeftMotor();
      auto trMotor = model->getTopRightMotor();

      for (auto motor : {blMotor, brMotor, tlMotor, trMotor}) {
        motor->setBrakeMode(chassis_break ? AbstractMotor::brakeMode::hold
                                          : AbstractMotor::brakeMode::coast);
      }

      pros::lcd::set_text(3, "Chassis Break: " + std::to_string(chassis_break));
    }

    // toggle intake
    BUTTON(pros::E_CONTROLLER_DIGITAL_R1) {
      if (intake_motor->getTargetVelocity() == 0) {
        intake_motor->moveVelocity(600);
      } else {
        intake_motor->moveVelocity(0);
      }
    }

    // intake lcd: Intake <actual velocity> <target velocity>
    pros::lcd::set_text(
        4, "Intake " + std::to_string(intake_motor->getActualVelocity()) + " " +
               std::to_string(intake_motor->getTargetVelocity()));

    // toggle flywheel
    BUTTON(pros::E_CONTROLLER_DIGITAL_L1) {
      switch (catapult_state) {
      case REELING:
      case LAUNCHING:
        // pass
        break;

      case IDLE:
        // set state to REELING
        catapult_state = REELING;
        catapult_motor->moveVelocity(100);
        pros::lcd::set_text(6, "Catapult: REELING BACK");
        break;

      case READY_TO_LAUNCH:
        // set state to LAUNCHING
        catapult_state = LAUNCHING;
        catapult_motor->moveVelocity(100);
        break;
      }
    }

    // display potentiometer value and catapult_status
    pros::lcd::set_text(
        5, "Potentiometer: " + std::to_string(catapult_pot->get_value()) +
               " Catapult: " + std::to_string(catapult_state));

    // print actual flywheel speed on 4th line of LCD
    // pros::lcd::set_text(
    //     4, "Flywheel: " + std::to_string(flywheel->getActualVelocity()) +
    //            " rpm (" + std::to_string(flywheel->getTemperature()) + "C)");

    // delay
    pros::delay(10);
  }
}