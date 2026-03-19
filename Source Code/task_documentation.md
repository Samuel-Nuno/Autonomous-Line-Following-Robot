# Task-Level Software Documentation

## High-Level Overview
The robot software is organized as a cooperative multitasking system using a priority-based scheduler. Each subsystem (motor control, sensing, user interface, etc.) is implemented as an independent task. Tasks communicate using shared variables (shares) and queues to ensure modularity and non-blocking execution.

At runtime:
- The scheduler repeatedly calls each task based on priority and timing
- Tasks update sensors, compute control actions, and send commands
- Shared variables and queues allow safe communication between tasks

This structure allows simultaneous operation of sensing, control, and decision-making without using interrupts or threads.

## Task Diagram
![Task Diagram](PASTE_TASK_DIAGRAM_IMAGE_HERE)

---

## Shared Data Table

| Data Name | Data Type | Share / Queue | Description |
|----------|----------|--------------|------------|
| line_position | float/int | Share | line centroid |
| line_valid | bool | Share | line detected |
| bump_state | bool | Share | bump pressed |
| left_velocity | float | Share | left speed |
| right_velocity | float | Share | right speed |
| left_setpoint | float | Share | left target |
| right_setpoint | float | Share | right target |
| imu_heading | float | Share | robot heading |
| user_command | string/int | Queue | user input |

---

## Task Descriptions and FSMs

### Task: Scheduler / Main Task
#### Overview
The main task is responsible for setting up the entire software system before normal robot operation begins. It initializes the hardware interfaces, configures pins and timers, creates the shared variables and queues used for inter-task communication, and instantiates each task object with its assigned priority and execution period. Once initialization is complete, the main section starts the cooperative scheduler, which repeatedly runs each task at the appropriate time. In this way, the main task acts as the entry point and system organizer for the entire robot program.

---

### Task: Line Sensor Task
#### Overview
The line sensor task is responsible for reading the analog reflectance sensor array and converting the raw sensor values into useful line-position information for the controller. It manages sensor calibration on both light and dark surfaces, normalizes the sensor readings so they can be compared consistently, and computes a weighted centroid that estimates the position of the line across the array. This task also determines whether a valid line is currently detected, which is important for both normal line following and loss-of-line recovery behavior. By isolating all line-sensing operations in one task, the rest of the system can use a clean and consistent line-position signal without needing to process raw sensor data directly.

#### Finite State Machine
![Line Sensor FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: Closed Loop Control Task
#### Overview
The closed-loop control task serves as the main decision-making and motion-control task for the robot. It reads the latest sensor information, including line position, wheel-speed feedback, bump status, and recovery conditions, and uses that information to compute the motor commands needed for autonomous behavior. During normal operation, it applies PI control to regulate left and right wheel velocities while also using the line-position error to generate smooth steering corrections for line following. This task is also responsible for detecting when the line has been lost and switching into scripted recovery behavior, such as driving forward, turning, and searching until the line is found again. Because it combines motion control with behavior logic, this task is central to how the robot reacts to both expected path-following conditions and off-nominal events.

#### Finite State Machine
![Closed Loop FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: Left Drive System
#### Overview
The left drive system task manages the complete low-level operation of the left wheel. It updates the encoder reading to determine wheel position and velocity, applies the commanded motor effort through the motor driver, and provides current feedback values to the rest of the software through shared data. This task ensures that the left side of the drivetrain responds consistently to controller commands and that accurate motion information is always available for closed-loop control. By separating the left wheel into its own task, the software remains modular and allows left-side sensing and actuation to be handled independently from the rest of the robot.

#### Finite State Machine
![Left Drive FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: Right Drive System
#### Overview
The right drive system task performs the same role for the right wheel that the left drive system task performs for the left. It continuously updates the encoder measurement for the right side, determines the wheel’s current position and velocity, and applies the requested motor command through the motor driver hardware. The measured motion data is then shared with the control system so that wheel-speed regulation and straight-line behavior can be maintained. Keeping the right drivetrain logic in a dedicated task makes the code easier to test, debug, and tune, especially when comparing the performance of the two sides of the robot.

#### Finite State Machine
![Right Drive FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: Bump Sensor Task
#### Overview
The bump sensor task monitors the robot’s single limit switch and converts its physical contact state into a software signal that can be used by higher-level control logic. Its purpose is to detect when the robot has made contact with an obstacle, wall, or other boundary condition that requires a response. The task updates the shared bump-state variable so the controller can quickly react, such as by interrupting normal line following and entering a recovery or repositioning sequence. Since switch inputs can change rapidly, this task also provides a clean and isolated way to handle repeated sampling of the sensor without mixing that logic into the main controller.

#### Finite State Machine
![Bump FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: IMU Heading Task
#### Overview
The IMU heading task reads orientation data from the BNO055 inertial measurement unit and updates the robot’s heading information for use by the rest of the system. Its main role is to provide an absolute or incremental measure of yaw so the robot can determine how far it has turned during maneuvers. This is especially useful during recovery routines and heading-based turns, where accurate rotation is needed and time-based open-loop turning would be less reliable. By assigning IMU processing to its own task, heading measurements can be updated regularly and made available to control logic without cluttering the main behavioral code.

#### Finite State Machine
![IMU FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: User Interface Task
#### Overview
The user interface task provides a way for the operator to interact with the robot through the serial terminal. It is used for testing, debugging, and tuning by receiving user commands and displaying useful system information such as sensor readings, control states, or performance data. This task makes it easier to adjust parameters, trigger behaviors, or monitor the robot during development without changing the main control structure. By isolating terminal interaction in its own task, the robot can support debugging and user commands while still maintaining organized real-time operation across the rest of the software.

#### Finite State Machine
![User FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: Line Sensor Task
#### Overview
Reads the line sensor array, performs calibration and normalization, and computes the centroid position of the line.

#### Finite State Machine
![Line Sensor FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: Closed Loop Control Task
#### Overview
Implements PI control for wheel velocities and line-following logic. Generates motor setpoints based on sensor inputs.

#### Finite State Machine
![Closed Loop FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: Left Drive System
#### Overview
Controls the left motor using PWM and encoder feedback. Tracks position and velocity.

#### Finite State Machine
![Left Drive FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: Right Drive System
#### Overview
Controls the right motor using PWM and encoder feedback. Tracks position and velocity.

#### Finite State Machine
![Right Drive FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: Bump Sensor Task
#### Overview
Monitors the limit switch and updates bump state for obstacle detection.

#### Finite State Machine
![Bump FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: IMU Heading Task
#### Overview
Reads IMU data and updates robot heading for orientation-based control.

#### Finite State Machine
![IMU FSM](PASTE_FSM_IMAGE_HERE)

---

### Task: User Interface Task
#### Overview
Handles user input and debugging through serial interface. Allows parameter tuning and monitoring.

#### Finite State Machine
![User FSM](PASTE_FSM_IMAGE_HERE)

