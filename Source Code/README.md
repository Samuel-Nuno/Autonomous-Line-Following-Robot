# ME405 Romi Robot Control System

## High Level Overview
This project implements a cooperative multitasking control system for a Romi robot using MicroPython. The robot uses multiple concurrent tasks to read sensors, control both motors, follow a line, and interact with the user through a serial interface.

At a high level, the system works as follows:
- The **main file** creates all shares, queues, and task objects
- The **drive system tasks** read encoders and apply motor effort
- The **closed loop task** performs PI velocity control and line-following logic
- The **line sensor task** computes the line centroid
- The **bump task** detects physical contact with an obstacle
- The **IMU heading task** reads robot heading and yaw rate
- The **user task** provides a serial command interface for tuning and testing
- The **ME405 cotask/task_share libraries** handle task scheduling and safe inter-task communication

This design separates sensing, control, and user interaction into modular finite state machines that communicate through shared variables and queues.

---

## Task Diagram
![Insert task diagram image here](https://github.com/Samuel-Nuno/Autonomous-Line-Following-Robot/blob/main/Reference%20Documentation/Images/User%20Task%20FSM.jpeg)

---

## Main File Overview
The `main.py` file is responsible for setting up the entire robot software system. It:
- Creates all shared variables (`Share`) and data buffers (`Queue`)
- Initializes default values such as gains, setpoints, and sensor outputs
- Creates each task object
- Adds each task to the scheduler with a priority and execution period
- Continuously runs the priority scheduler inside the main loop

The file defines shares for:
- left and right motor enable flags
- left and right velocity setpoints
- left and right measured velocities
- left and right encoder positions
- left and right control efforts
- PI controller gains
- line-following enable, speed, and centroid
- bump sensor state

The file also creates queues for logged time, velocity, and line centroid data. Once all tasks are created, they are appended to the task list and scheduled with `task_list.pri_sched()`.

---

# Tasks

## Left and Right drivesystem file

### Overview
The left and right drive system files implement the motor-side tasks for each wheel. These tasks are responsible for:
- reading the encoder every cycle
- computing wheel position and wheel velocity
- writing measured velocity to a shared variable
- writing encoder position to a shared variable
- reading the desired control effort from the closed-loop controller
- applying motor effort to the physical motor driver
- logging time and velocity data into queues for later display

These two files are nearly identical in structure, but they use different hardware pins and timers for the left and right motors.

### FSM
![Insert Left/Right Drive System FSM image here](https://github.com/Samuel-Nuno/Autonomous-Line-Following-Robot/blob/main/Reference%20Documentation/Images/Drive%20System%20FSM.jpg)

---

## Closed Loop file

### Overview
The `ClosedLoop.py` file contains the main controller for the robot. This task performs the closed-loop velocity control for both motors and also contains the higher-level line-following behavior.

Its main responsibilities are:
- reading left and right setpoints
- reading measured left and right wheel velocities
- computing velocity error for each wheel
- applying PI control to generate motor effort commands
- saturating the output effort to safe limits
- resetting integrators when setpoints change significantly
- supporting line-following operation by converting centroid error into left/right wheel setpoints
- handling lost-line behavior through a multi-step garage recovery script

When line following is enabled, the controller does not simply use externally supplied setpoints. Instead, it uses the line centroid and desired forward speed to generate left and right setpoints automatically. If the line is lost for long enough, the controller enters a scripted recovery routine that drives forward, turns, and searches for the line again.

### FSM
![Insert Closed Loop FSM image here](https://github.com/Samuel-Nuno/Autonomous-Line-Following-Robot/blob/main/Reference%20Documentation/Images/Closed%20Loop%20FSM.jpg)

---

## Task_User File

### Overview
The `task_user.py` file implements the user interface task. It communicates with a PC through USB serial and allows the user to interact with the robot at runtime.

This task allows the user to:
- print a help menu
- calibrate the line sensor on white and black surfaces
- set new proportional and integral gains
- set new left and right wheel setpoints
- enable or disable line following
- enter a line-following base velocity
- print raw and normalized line sensor values
- trigger a step response and print logged data
- emergency stop the robot
- print estimated pose values if estimator shares are connected
- reset pose values if an estimator task is connected

The user task is the main tuning and debugging interface for the robot.

### FSM
![Insert Task_User FSM image here](https://github.com/Samuel-Nuno/Autonomous-Line-Following-Robot/blob/main/Reference%20Documentation/Images/User%20Task%20FSM.jpeg)

---

## task_bump

### Overview
The `task_bump.py` file reads the bump sensor and publishes a debounced pressed/not-pressed result to a shared variable.

Its main responsibilities are:
- configuring the bump input pin
- handling either active-low or active-high wiring
- sampling the bump signal periodically
- debouncing the switch input over several samples
- publishing a stable bump state to a shared variable

The bump state is used by the closed-loop controller during garage recovery behavior.

### FSM
*[Insert task_bump FSM image here]*

---

## task_imu_heading

### Overview
The `task_IMU_heading.py` file manages heading measurements from the BNO055 IMU. It initializes the IMU, stores a zero-heading reference, and continuously publishes heading and yaw rate.

Its main responsibilities are:
- initializing the I2C interface and IMU driver
- setting the BNO055 operating mode
- reading the absolute heading from the IMU
- subtracting the startup heading to create a relative heading
- reading yaw rate from the gyro
- writing heading and heading rate to shared variables
- allowing the heading reference to be reset

This task is useful for orientation measurement and future closed-loop heading control.

### FSM
*[Insert task_imu_heading FSM image here]*

---

## task_line_sensor

### Overview
The `task_line_sensor.py` file reads the 8-channel line sensor and computes the line centroid used for line following.

Its main responsibilities are:
- creating the line sensor driver object
- reading the line only when line following is enabled
- computing the centroid position from calibrated sensor values
- outputting a centroid value from 0 to 7 when a line is found
- outputting `-1.0` when no valid line is detected

This task isolates line detection from the main closed-loop controller and keeps the design modular.

### FSM
*[Insert task_line_sensor FSM image here]*

---

# Drivers

## Encoder

### Overview
The `encoder.py` file implements a quadrature encoder interface. It uses a hardware timer in encoder mode to track wheel position and wheel speed.

### What it sets up
The encoder driver sets up:
- a hardware timer in encoder mode
- channel A and channel B encoder inputs
- internal variables for position, previous count, delta count, and elapsed time

### What data it takes in
The driver takes in:
- a timer number
- a pin for encoder channel A
- a pin for encoder channel B

### Methods
- `__init__(tim, chA_pin, chB_pin)`  
  Creates the encoder object, initializes the timer in encoder mode, and prepares all internal variables.

- `update()`  
  Reads the current timer count, computes the count difference since the last update, corrects for timer overflow, updates the cumulative position, and computes elapsed time.

- `get_position()`  
  Returns the total encoder position in counts.

- `get_velocity()`  
  Returns the wheel velocity in mm/s using the most recently computed count delta and elapsed time.

- `zero()`  
  Resets the position to zero and updates the internal reference count.

---

## Motor

### Overview
The `motor.py` file provides a motor driver interface for a DC motor driver that uses a PWM pin, a direction pin, and a sleep pin.

### What it sets up
The motor driver sets up:
- a PWM output pin
- a direction output pin
- a sleep/enable output pin
- a timer PWM channel for duty-cycle control

### What data it takes in
The driver takes in:
- PWM pin
- direction pin
- sleep pin
- timer object
- timer channel number
- requested motor effort from `-100` to `100`

### Methods
- `__init__(PWM, DIR, nSLP, tim, channel)`  
  Creates the motor object and configures the PWM, direction, and sleep pins.

- `set_effort(effort)`  
  Applies a signed effort command. Positive values drive in one direction, negative values drive in the opposite direction, and out-of-range values result in zero PWM.

- `enable()`  
  Wakes the motor driver from sleep mode and sets PWM output to zero.

- `disable()`  
  Puts the motor driver into sleep mode.

---

## linesensor

### Overview
The `linesensor.py` file provides a driver for the 8-channel Pololu QTR-MD-08A analog line sensor. It reads raw ADC values, normalizes them using calibration data, and computes a weighted centroid.

### What it sets up
The line sensor driver sets up:
- 8 ADC inputs
- white calibration values
- black calibration values
- sensor index positions from 0 to 7
- thresholds for line detection robustness

### What data it takes in
The driver takes in:
- a list of 8 ADC-capable pins

It also uses live ADC readings from each sensor channel.

### Methods
- `__init__(adc_pins)`  
  Creates ADC objects for all 8 sensors and initializes calibration data.

- `read_raw()`  
  Returns the raw ADC readings from all 8 sensors.

- `calibrate_white(n=50)`  
  Samples the sensor multiple times over a white surface and stores average white calibration values.

- `calibrate_black(n=50)`  
  Samples the sensor multiple times over a black line and stores average black calibration values.

- `norm_from_raw(raw)`  
  Converts raw readings into normalized values between 0 and 1 using calibration data.

- `read_norm()`  
  Reads raw sensor values and returns normalized values.

- `centroid_from_norm(w)`  
  Computes the weighted centroid from normalized sensor values. Returns `None` if no confident line is detected.

- `centroid()`  
  Performs a full raw read, normalization, and centroid computation in one step.

- `read_all()`  
  Returns raw values, normalized values, and centroid together for debugging.

---

## IMU_Driver

### Overview
The `IMU_Driver.py` file implements an I2C driver for the BNO055 inertial measurement unit. It supports heading, Euler angles, gyro data, calibration status, and saving/loading calibration constants.

### What it sets up
The IMU driver sets up:
- the I2C interface reference
- the IMU address
- an optional reset pin
- register-level communication helpers
- IMU operating mode and calibration functions

### What data it takes in
The driver takes in:
- a previously created I2C object
- an optional I2C address
- an optional reset pin

It reads data from:
- heading registers
- roll and pitch registers
- gyro registers
- calibration registers
- chip ID and configuration registers

### Methods
- `__init__(i2c, address=DEFAULT_ADDR, rst_pin=None)`  
  Creates the IMU driver object and stores references to the I2C bus and optional reset pin.

- `read_chip_id()`  
  Reads the BNO055 chip ID register.

- `hw_reset()`  
  Applies a hardware reset using the reset pin if one is connected.

- `set_mode(mode)`  
  Changes the BNO055 operating mode, such as IMU mode or NDOF mode.

- `get_calib_status()`  
  Returns the system, gyro, accelerometer, and magnetometer calibration status.

- `read_calibration_data()`  
  Reads the 22-byte calibration block from the IMU.

- `write_calibration_data(calib_bytes)`  
  Writes a 22-byte calibration block back to the IMU.

- `save_calibration_to_file(filename="bno055_calib.bin")`  
  Saves calibration bytes to a file.

- `load_calibration_from_file(filename="bno055_calib.bin")`  
  Loads calibration bytes from a file and writes them to the IMU.

- `read_euler()`  
  Returns heading, roll, and pitch in degrees.

- `read_heading()`  
  Returns just the heading in degrees.

- `read_gyro()`  
  Returns x, y, and z gyro rates in degrees per second.

- `read_yaw_rate_dps()`  
  Returns yaw rate in degrees per second.

- `read_yaw_rate_rads()`  
  Returns yaw rate in radians per second.

---

# [ME405 Library Files](https://github.com/spluttflob/ME405-Support)

## task_share

### Overview
The `task_share.py` file provides safe communication objects for sharing data between tasks. It contains two main classes:
- `Share` for a single shared value
- `Queue` for buffered FIFO data transfer

These classes are designed to reduce data corruption risk when tasks exchange data, especially when interrupts may be involved.

The file also maintains a system-wide list of all shares and queues for diagnostics.

---

## cotask

### Overview
The `cotask.py` file provides the cooperative multitasking framework used by the project.

Its main purpose is to:
- define tasks as generators
- allow tasks to yield control back to the scheduler
- schedule tasks based on period and priority
- support both round-robin and priority-based scheduling
- optionally profile execution timing
- optionally trace state transitions

The two main classes are:
- `Task` for representing an individual scheduled task
- `TaskList` for storing tasks and running the scheduler

This file is the foundation that allows all of the robot tasks to run together without a full RTOS.
