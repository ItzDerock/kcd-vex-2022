#include "main.h"
#include "controllers/movement/odom.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "util.hpp"

#include "controllers/auton/auton.hpp"
#include "controllers/movement/movement.hpp"
#include "core/config.hpp"

int DANGEROUS_OVERRIDE_LAST_PRESS = -1;
bool endgame_launching = false;

enum DRIVE_MODE { FIELD_CENTERED, ROBOT_CENTERED };
DRIVE_MODE drive_mode = FIELD_CENTERED;

#define CATAPULT_POT_LOADING 2723
#define CATAPULT_POT_LAUNCHED 1100

void endgame() {
  if (endgame_launching)
    return;
  endgame_launching = true;

  if (catapult_state == DISABLED) {
    endgame_launcher->set_value(1);
  } else if (catapult_state != IDLE) {
    catapult_state = DISABLED;

    // move until 1200 on potentiometer
    catapult_motor->moveVelocity(100);
    while (catapult_pot->get_value() > CATAPULT_POT_LAUNCHED) {
      pros::delay(10);
    }
    catapult_motor->moveVelocity(0);

    pros::delay(1500);

    endgame_launcher->set_value(1);
  }

  endgame_launching = false;
}

void run_catapult() {
  if (catapult_state == IDLE && auto_reload) {
    catapult_state = REELING;
  }

  // only run if catapult status is "REELING"
  if (catapult_state == REELING) {
    // move until 1200 on potentiometer
    if (catapult_pot->get_value() < CATAPULT_POT_LOADING) {
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
      if (catapult_pot->get_value() > CATAPULT_POT_LAUNCHED) {
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

  pros::lcd::register_btn0_cb(auton::toggleMode);
  pros::lcd::register_btn1_cb(auton::toggleSide);
  auton::updateDisplay();

  // button 3 should run auton if not in competition mode
  pros::lcd::register_btn2_cb([]() {
    if (!pros::competition::is_connected()) {
      auton::run();
    }
  });

  pros::lcd::set_text(1, "[i] Calibrating IMU and POT...");

  // calibrate IMU
  while (inertial->is_calibrating()) {
    if (errno != 0) {
      pros::lcd::set_text(1, "[!] IMU Error " + std::to_string(errno));
      printf("IMU Error %s (%d)", strerror(errno), errno);
    } else {
      printf("IMU Calibrating...");
    }

    pros::delay(10);
  }

  // tare
  inertial->tare();
  catapult_motor->setBrakeMode(AbstractMotor::brakeMode::hold);

  // calibrate catapult potentiometer
  catapult_pot->calibrate();

  // start task to update position on screen
  // pros::Task updatePositionOnScreenTask(movement:: updatePositionLoop);
  pros::Task odometry(odom::run);

  pros::lcd::set_text(1, "[i] Ready to rumble!");
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
  // movement::resetPosition();
  pros::lcd::set_text(1, "[i] Robot Disabled");
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
  // recalibrate IMU
  pros::lcd::set_text(1, "[i] Recalibrating IMU (competition)...");

  catapult_pot->calibrate();

  while (inertial->is_calibrating()) {
    pros::delay(10);
  }

  inertial->tare();

  odom::reset();

  pros::lcd::set_text(1, "[i] Ready to rumble!");
}

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
  auton::run();
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

    // position updater
    // movement::updatePosition();

    // get joystick values
    double irightSpeed = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
    double iforwardSpeed = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
    double irotSpeed = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

    // scale joystick values
    double rightSpeed = utils::mapValue(irightSpeed, -127, 127, -1, 1);
    double forwardSpeed = utils::mapValue(iforwardSpeed, -127, 127, -1, 1);
    double rotationSpeed = utils::mapValue(irotSpeed, -127, 127, -1, 1);

    // deadzone, if all values are below 0.1 return
    if (fabs(rightSpeed) < 0.01 && fabs(forwardSpeed) < 0.01 &&
        fabs(rotationSpeed) < 0.01) {
      pros::lcd::set_text(3, "Chassis: Stopped");
      chassis->stop();
    } else {
      if (drive_mode == FIELD_CENTERED) {
        // field centered driving
        // convert the values using the heading by imu.get_heading();
        double heading = inertial->get_heading();

        // convert input to radians
        heading *= (M_PI / 180);

        // calculate the new rightSpeed and forwardSpeed
        double temp = rightSpeed * cos(heading) - forwardSpeed * sin(heading);
        forwardSpeed = rightSpeed * sin(heading) + forwardSpeed * cos(heading);
        rightSpeed = temp;
      }

      // set chassis speed
      model->xArcade(rightSpeed, forwardSpeed, rotationSpeed);

      // print right, forward, rot on 2nd line of LCD
      pros::lcd::set_text(3, "R: " + utils::round(rightSpeed, 2) +
                                 " Fw: " + utils::round(forwardSpeed, 2) +
                                 " Rt: " + utils::round(rotationSpeed, 2));
    }

    // toggle chassis break
    BUTTON(pros::E_CONTROLLER_DIGITAL_B) {
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

      pros::lcd::set_text(4, "Chassis Break: " + std::to_string(chassis_break));
    }

    // BUTTON(pros::E_CONTROLLER_DIGITAL_R2) { movement::updatePositionLoop(); }

    // toggle intake
    BUTTON(pros::E_CONTROLLER_DIGITAL_R1) {
      if (intake_motor->getTargetVelocity() == 0 ||
          intake_motor->getTargetVelocity() == -200) {
        intake_motor->moveVelocity(200);
      } else {
        intake_motor->moveVelocity(0);
      }
    }

    // reverse intake
    BUTTON(pros::E_CONTROLLER_DIGITAL_R2) {
      if (intake_motor->getTargetVelocity() == 0 ||
          intake_motor->getTargetVelocity() == 200) {
        intake_motor->moveVelocity(-200);
      } else {
        intake_motor->moveVelocity(0);
      }
    }

    // intake lcd: Intake <actual velocity> <target velocity>
    pros::lcd::set_text(
        5, "Intake " + std::to_string(intake_motor->getActualVelocity()) + " " +
               std::to_string(intake_motor->getTargetVelocity()));

    // toggle flywheel
    BUTTON(pros::E_CONTROLLER_DIGITAL_Y) {
      switch (catapult_state) {
      case REELING:
      case LAUNCHING:
      case DISABLED:
        // pass
        break;

      case IDLE:
        // set state to REELING
        catapult_state = REELING;
        catapult_motor->moveVelocity(100);
        break;

      case READY_TO_LAUNCH:
        // spin intake_motor until multiple of 360
        // auto needed = 360 - std::fmod(intake_motor->getPosition(), 360);
        // intake_motor->moveRelative(needed, 200);

        // set state to LAUNCHING
        catapult_state = LAUNCHING;
        catapult_motor->moveVelocity(100);
        break;
      }
    }

    // toggle roller
    BUTTON(pros::E_CONTROLLER_DIGITAL_L2) {
      if (roller_motor->getTargetVelocity() == 0) {
        roller_motor->moveVelocity(100);
      } else {
        roller_motor->moveVelocity(0);
      }
    }

    // toggle auto-reload
    BUTTON(pros::E_CONTROLLER_DIGITAL_X) {
      auto_reload = !auto_reload;
      pros::lcd::set_text(7, "Auto Reload: " + std::to_string(auto_reload));
    }

    // launch endgame
    BUTTON(pros::E_CONTROLLER_DIGITAL_LEFT) {
      endgame();
      // run endgame() as a task
      // auto task = pros::Task(endgame);
      // printf("endgame task started: %d", task.get_state());
    }

    // disable catapult
    BUTTON(pros::E_CONTROLLER_DIGITAL_RIGHT) {
      if (pros::millis() - DANGEROUS_OVERRIDE_LAST_PRESS < 1000) {
        // if pressed twice within 1 second, disable catapult
        catapult_state = DISABLED;
        catapult_motor->moveVelocity(0);

        // rumble controller
        master.rumble(".--");
      } else {
        // if pressed once, set last press to current time
        DANGEROUS_OVERRIDE_LAST_PRESS = pros::millis();
      }
    }

    if (catapult_state == DISABLED) {
      // up/down manual override
      HELD(pros::E_CONTROLLER_DIGITAL_UP) { catapult_motor->moveVelocity(100); }
      else HELD(pros::E_CONTROLLER_DIGITAL_DOWN) {
        catapult_motor->moveVelocity(-100);
      }
      else {
        catapult_motor->moveVelocity(0);
      }
    }

    // toggle field mode
    BUTTON(pros::E_CONTROLLER_DIGITAL_A) {
      if (drive_mode == FIELD_CENTERED) {
        drive_mode = ROBOT_CENTERED;
      } else {
        drive_mode = FIELD_CENTERED;
      }
    }

    // display potentiometer value and catapult_status
    pros::lcd::set_text(
        6, "Potentiometer: " + std::to_string(catapult_pot->get_value()) +
               " Catapult: " + std::to_string(catapult_state));

    // print actual flywheel speed on 4th line of LCD
    // pros::lcd::set_text(
    //     4, "Flywheel: " + std::to_string(flywheel->getActualVelocity()) +
    //            " rpm (" + std::to_string(flywheel->getTemperature()) + "C)");

    // delay
    pros::delay(10);
  }
}
