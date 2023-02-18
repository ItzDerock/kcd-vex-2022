#include "main.h"
#include "controllers/movement/odom.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "util.hpp"

#include "controllers/auton/auton.hpp"
#include "controllers/movement/movement.hpp"
#include "core/config.hpp"

// variables to handle double clicking.
int DANGEROUS_OVERRIDE_LAST_PRESS = -1;
int l1_last_press = -1;
int a_last_press = -1;

// handles the drive mode
enum DRIVE_MODE { FIELD_CENTERED, ROBOT_CENTERED };
DRIVE_MODE drive_mode = FIELD_CENTERED;

// launches the endgame.
void endgame() { endgame_launcher->set_value(1); }

// Handles the catapult
// Should be ran recursively
void run_catapult() {
  if (catapult_state == DISABLED)
    return;

  // printf("%d", catapult_state);
  if (catapult_state == IDLE && auto_reload) {
    catapult_state = REELING;
  }

  // if reeling
  if (catapult_state == REELING || catapult_state == LAUNCHING) {
    if (intake_holder->getTargetPosition() == 0) {
      intake_holder->moveAbsolute(30, 600);
    }
  } else if (catapult_state == READY_TO_LAUNCH) {
    if (intake_holder->getTargetPosition() == 30) {
      intake_holder->moveAbsolute(0, 600);
    }
  }

  // only run if catapult status is "REELING"
  if (catapult_state == REELING) {
    // move until 1200 on potentiometer
    if (catapult_pot->get_value() < CATAPULT_POT_LOADING) {
      if (catapult_motor->getTargetVelocity() == 0) {
        catapult_motor->moveVelocity(75);
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

// The main catapult loop
// Should be invoked in a pros::Task
void catapult_task() {
  while (true) {
    run_catapult();

    // if (catapult_state == DISABLED)
    //   break;

    pros::delay(20);
  }

  printf("Warning: cata task ended");
}

// pros::Task catapult(catapult_task);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();

  // initialize the autonomous selector buttons.
  pros::lcd::register_btn0_cb(auton::toggleMode);
  pros::lcd::register_btn1_cb(auton::toggleSide);
  auton::updateDisplay();

  // button 3 should run auton if not in competition mode
  // for debugging
  pros::lcd::register_btn2_cb([]() {
    if (!pros::competition::is_connected()) {
      auton::run();
    }
  });

  // Calibrate everything
  pros::lcd::set_text(1, "[i] Calibrating IMU and POT...");

  // calibrate IMU
  while (inertial->is_calibrating()) {
    if (errno != 0) {
      // log to screen so this is visible without a serial connection.
      pros::lcd::set_text(1, "[!] IMU Error " + std::to_string(errno));
      printf("IMU Error %s (%d)", strerror(errno), errno);
    } else {
      printf("IMU Calibrating...");
    }

    pros::delay(10);
  }

  // tare other sensors
  inertial->tare();

  // this is important, makes sure our catapult is set to the hold brake mode.
  catapult_motor->setBrakeMode(AbstractMotor::brakeMode::hold);
  intake_holder->setBrakeMode(AbstractMotor::brakeMode::hold);

  // calibrate catapult potentiometer
  catapult_pot->calibrate();

  // Start odometry tracking
  pros::Task odometry(odom::run);
  // chassis->startOdomThread();

  // finally, update a message on the screen so we know we are good to go.
  pros::lcd::set_text(1, "[i] Ready to rumble!");
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
void competition_initialize() {
  // recalibrate everything
  pros::lcd::set_text(1, "[i] Recalibrating IMU (competition)...");

  // potentiometer
  catapult_pot->calibrate();

  // inertial
  while (inertial->is_calibrating()) {
    pros::delay(10);
  }

  // reset heading
  inertial->tare();

  // reset to 0,0 position.
  odom::reset();

  // and we're ready!
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
  // hand it off to src/controllers/auton/auton.cpp
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

  // revive catapult task
  // printf("Catapult state is %d", catapult.get_state());
  // if (catapult.get_state() == pros::E_TASK_STATE_DELETED) {
  //   catapult = pros::Task(catapult_task);
  // }
  // UNCOMMENT IF WE GO BACK TO CATAPULT AS A TASK.

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

    // deadzone, if all values are below 0.01 ignore the input.
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
    BUTTON(pros::E_CONTROLLER_DIGITAL_B) { movement::toggleChassisBreak(); }

    // toggle intake
    BUTTON(pros::E_CONTROLLER_DIGITAL_R1) {
      // if the intake speed is 0, then intake is off.
      // turn it on
      if (intake_motor->getTargetVelocity() == 0 ||
          intake_motor->getTargetVelocity() == -300) {
        intake_motor->moveVelocity(300);
      } else {
        intake_motor->moveVelocity(0);
      }
    }

    // reverse intake
    BUTTON(pros::E_CONTROLLER_DIGITAL_R2) {
      // same logic as above.
      if (intake_motor->getTargetVelocity() == 0 ||
          intake_motor->getTargetVelocity() == 300) {
        intake_motor->moveVelocity(-300);
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
      // update the catapult state to the new state
      switch (catapult_state) {
      case REELING:
      case LAUNCHING:
      case DISABLED:
        // pass, run_catapult() will handle it.
        break;

        // IDLE means we are in a position to start reeling back.
      case IDLE:
        // set state to REELING and let run_catapult() handle it next tick
        catapult_state = REELING;
        catapult_motor->moveVelocity(100);
        break;

        // READY_TO_LAUNCH means we can launch.
      case READY_TO_LAUNCH:
        // set state to LAUNCHING. run_catapult() will handle it after this.
        catapult_state = LAUNCHING;
        catapult_motor->moveVelocity(100);
        break;
      }
    }

    // toggle the roller motor
    BUTTON(pros::E_CONTROLLER_DIGITAL_L2) {
      if (roller_motor->getTargetVelocity() == 0) {
        roller_motor->moveVelocity(100);
      } else {
        roller_motor->moveVelocity(0);
      }
    }

    // toggle auto-reload in case we ever need it off.
    BUTTON(pros::E_CONTROLLER_DIGITAL_X) {
      auto_reload = !auto_reload;
      pros::lcd::set_text(7, "Auto Reload: " + std::to_string(auto_reload));
    }

    // launch endgame
    BUTTON(pros::E_CONTROLLER_DIGITAL_LEFT) { endgame(); }

    // disable catapult
    BUTTON(pros::E_CONTROLLER_DIGITAL_RIGHT) {
      // if the catapult is disabled, this button will toggle the intake holder
      // if (catapult_state == DISABLED) {
      //   // toggle mode
      //   intake_holder->moveVelocity(
      //       intake_holder->getTargetPosition() == 30 ? 0 : 30);
      // }

      if (pros::millis() - DANGEROUS_OVERRIDE_LAST_PRESS < 1000) {
        // if pressed twice within 1 second, disable catapult
        // this will set the catapult to manual control
        // only use when catapult is malfunctioning or is stuck.
        if (catapult_state != DISABLED) {
          catapult_state = DISABLED;
          catapult_motor->moveVelocity(0);
        } else {
          catapult_state = IDLE;
        }

        // rumble controller so we know this happened.
        master.rumble(".--");
      } else {
        // if pressed once, set last press to current time
        DANGEROUS_OVERRIDE_LAST_PRESS = pros::millis();

        // rumble as a warning
        master.rumble("-");
      }
    }

    // set home location for moving
    BUTTON(pros::E_CONTROLLER_DIGITAL_L1) {
      printf("single press, last press: %d", l1_last_press);
      // if double press, set the location
      if (pros::millis() - l1_last_press < 200) {
        // set home location
        movement::goalLocation = odom::globalPoint;
        master.rumble(".");
        pros::lcd::set_text(8, "Home Set");
        printf("dobule press, home is %f, %f", movement::goalLocation.x,
               movement::goalLocation.y);
        l1_last_press = -1;
      } else {
        // if single press, set last press to current time
        l1_last_press = pros::millis();
      }
    }
    else {
      // if it is a single press, look at home location
      if (l1_last_press != -1 && (pros::millis() - l1_last_press) > 200) {
        l1_last_press = -1;

        // calculate required angle
        double angle =
            std::atan2(movement::goalLocation.y - odom::globalPoint.y,
                       movement::goalLocation.x - odom::globalPoint.x);
        printf("atan2 returned %f (rad)\n", angle);
        angle = -angle + M_PI / 2;

        // turn to degrees
        angle = utils::getDegrees(angle);

        // make sure within range
        angle = std::fmod(angle + 360.0, 360.0);

        // face back
        angle = std::fmod(angle + 180.0, 360.0);

        // we want the back to face this direction
        // angle += 180;

        // if angle is not in [0, 360), make it
        // if (angle < 0) {
        // angle += 360;
        // } else if (angle >= 360) {
        // angle -= 360;
        // }

        printf("turning to %f\n", angle);
        printf("goal is at %f, %f\n", movement::goalLocation.x,
               movement::goalLocation.y);
        printf("current is at %f, %f\n", odom::globalPoint.x,
               odom::globalPoint.y);
        printf("errorX is %f\n",
               movement::goalLocation.x - odom::globalPoint.x);
        printf("errorY is %f\n",
               movement::goalLocation.y - odom::globalPoint.y);

        // look at that angle
        movement::turnTo(angle);
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

    // toggle field centered mode
    BUTTON(pros::E_CONTROLLER_DIGITAL_A) {
      // double press will tare heading
      if (pros::millis() - a_last_press < 200) {
        inertial->tare_heading();
        master.rumble("..");
        a_last_press = -1;
      } else {
        a_last_press = pros::millis();
      }
    }
    else {
      // otherwise, it is a single press.
      // toggle modes
      if (a_last_press != -1 && (pros::millis() - a_last_press) > 200) {
        a_last_press = -1;
        if (drive_mode == FIELD_CENTERED) {
          drive_mode = ROBOT_CENTERED;
        } else {
          drive_mode = FIELD_CENTERED;
        }
      }
    }

    // display potentiometer value and catapult_status
    pros::lcd::set_text(
        6, "Potentiometer: " + std::to_string(catapult_pot->get_value()) +
               " Catapult: " + std::to_string(catapult_state));

    // small delay so other tasks get some cpu cycles.
    pros::delay(10);
  }
}
