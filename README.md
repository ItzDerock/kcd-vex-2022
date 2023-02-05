# 76308A's Vex Spin Up Code
Won the "Think" award (best programming).  
Unfortnuately it wasn't a state-qualifying award.

Has the following amazing features
- X-Drive with 3 wheel odometry tracking.
- Autonomous selector.
- Field-orientated controls.
- Automatic catapult management.

Built using C++, PROS, and okapi.

## File Structure
`src/` contains all our code, broken down into a few sections.  
`utilities/` contains utility programs.

### `utilities/PIDPlotter.py`
Graphs the PID data in real time for debugging and tuning.
Shows 3 charts, one for x axis, one for y axis, one for the robot's heading.
Each chart contains the sensor reading, calculated error, and the PID loop's calculated output power.
Takes input data from the serial monitor.

Usage: `pros terminal | ./utilities/run.sh` (linux)

### `src/main.cpp`
The main bulk of our code, registers the autonomous function and contains the main driver-control code.
Handles all of the buttons and driving stuff for driver control.

### `src/util.hpp`
Contains a bunch of utility functions for math/trigonometry stuff. 

### `src/controllers/auton/*`
Contains files for our on-screen autonomous selector and the autonomous programs.

### `src/controllers/movement/movement.{hpp,cpp}`
Contains code regarding moving to a target point or turning to a certain angle using a PID (Proportional, Integral, Derivative) feedback loop.
Each axis gets its own PID controller, and the error is converted to a field-orientated manner and is then fed into the drivetrain model.

### `src/controllers/movement/odom.{hpp,cpp}`
Contains code related to our odometry system.

### `src/controllers/movement/pid.{hpp,cpp}`
Contains a utility class to manage the PID feedback loops.

### `src/core/config.hpp`
Contains all the port configuration stuff.

### `src/core/config.cpp`
Takes all the config.hpp stuff and sets up the drivetrain and motor classes for use in other files.
