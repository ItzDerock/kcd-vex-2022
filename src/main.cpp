#include "main.h"
#include "pros/llemu.hpp"
#include "util.hpp"

/**
 * Drive train ports
 */
#define DRIVE_TOP_LEFT 1
#define DRIVE_TOP_RIGHT 2
#define DRIVE_BOTTOM_LEFT 3
#define DRIVE_BOTTOM_RIGHT 4

#define INERTIAL_PORT 11

/**
 * Flywheel ports
 */
#define FLYWHEEL_1 20

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
 * define flywheel (blue gearset)
 */
auto flywheel = std::make_shared<okapi::Motor>(
    FLYWHEEL_1, true, AbstractMotor::gearset::blue,
    AbstractMotor::encoderUnits::degrees);

/**
 * states
 */
bool chassis_break = false;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "[i] Calibrating IMU...");

  // calibrate IMU
  inertial->calibrate();

  while (inertial->isCalibrating()) {
    pros::delay(10);
  }

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
    BUTTON(pros::E_CONTROLLER_DIGITAL_L1) {
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

    // toggle flywheel
    BUTTON(pros::E_CONTROLLER_DIGITAL_R1) {
      // get current flywheel speed
      double flywheel_speed = flywheel->getTargetVelocity();

      // if velocity is 0, set to 200 rpm
      if (flywheel_speed == 0) {
        flywheel->moveVelocity(600);
      } else {
        flywheel->moveVelocity(0);
      }
    }

    // print actual flywheel speed on 4th line of LCD
    pros::lcd::set_text(
        4, "Flywheel: " + std::to_string(flywheel->getActualVelocity()) +
               " rpm (" + std::to_string(flywheel->getTemperature()) + "C)");

    // delay
    pros::delay(10);
  }
}