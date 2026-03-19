# Autonomous-Line-Following-Robot
**Team Members:** Samuel Nuno, Bruce Berg

## Project Summary
![Project Summary Image](https://github.com/Samuel-Nuno/Autonomous-Line-Following-Robot/blob/main/Reference%20Documentation/Images/romi.jpg)

This project documents our ME 405 [Romi](https://www.pololu.com/category/202/romi-chassis-and-accessories) final project, in which we developed a differential-drive robot capable of autonomous line following with closed-loop wheel control, line position estimation, and obstacle-triggered recovery behavior. The goal of the project was to create a robot that could reliably follow the printed course at speed while using organized, reproducible software and clear system documentation.

## Project Objective
![Project Objective Image](https://github.com/Samuel-Nuno/Autonomous-Line-Following-Robot/blob/main/Reference%20Documentation/Images/track.jpg)

Our robot was designed to complete the Romi line-following time-trial by combining encoder-based wheel feedback, analog line sensing, and task-based control software. The system was structured to prioritize repeatable behavior and robustness rather than relying on open-loop tuning alone.

## Mechanical and Electrical Design
The platform is based on the Pololu Romi differential-drive chassis. The robot uses two independently driven wheels, each with its own motor and encoder feedback. In our preliminary design work, we identified several key chassis parameters that informed our control and sensing decisions, including a wheel radius of 35 mm and a track width of 141 mm.

![Romi Schematic](https://github.com/Samuel-Nuno/Autonomous-Line-Following-Robot/blob/main/Reference%20Documentation/Images/romidims.png)

The electrical system includes:
- Two DC drive motors
- Wheel encoders for each side
- An STM32 microcontroller running MicroPython
- A Pololu QTR-8A analog reflectance sensor array for line following
- A bump sensor implemented as a single limit switch
- Motor driver outputs using PWM and direction pins

### Pinout Diagram
![Pinout Diagram](PASTE_IMAGE_LINK_HERE)

### Motor and Encoder Interface
Each motor is controlled by a PWM signal and a direction pin. Encoder feedback is used to estimate wheel speed and wheel position continuously.

## Sensors Used

### [Wheel Encoders](https://www.pololu.com/product/3675)
We used one quadrature encoder per wheel to estimate wheel position and velocity. In software, encoder counts are converted into wheel speed in mm/s.

### [Line Sensor Array](https://www.pololu.com/product/4248)
We selected an 8-channel analog Pololu QTR-MD-08A reflectance sensor array. An analog sensor array was chosen because it provides continuous reflectance measurements across all channels, allowing for more precise line position estimation and smoother control compared to digital threshold-based sensors.

The line sensor is calibrated on both white background and black line surfaces. After calibration, the readings from each channel are normalized, and a weighted centroid is computed (ranging from 0 to 7) to estimate the position of the line relative to the sensor array.

### [Bump Sensor](https://www.pololu.com/product/1405)
The bump sensor is implemented as a **single limit switch**. This provides a simple and reliable method of detecting contact with obstacles or walls, allowing the robot to trigger recovery behavior when pressed.

### [IMU (Inertial Measurement Unit)](https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/overview)

The robot uses a BNO055 IMU to measure orientation, specifically the robot’s heading (yaw angle). The IMU provides absolute heading information, which allows the robot to understand its orientation relative to its starting direction.

Heading data is used primarily during recovery and maneuvering behaviors rather than continuous line following. When the robot performs actions such as turning 90 degrees or reorienting itself after losing the line, the IMU provides feedback to ensure accurate and repeatable rotations.

By monitoring the change in heading, the robot can:
- execute precise turns (e.g., 90° rotations),
- maintain consistent orientation during recovery sequences,
- and avoid relying solely on time-based or open-loop turning.

Using the IMU improves accuracy compared to open-loop turning because the robot can directly measure how far it has rotated rather than estimating based on motor commands alone.

## Software Architecture
The software is organized as a cooperative multitasking system using generators, scheduled by a priority-based task scheduler. Shared variables and queues are used for communication between tasks.

### Main software modules
- `main.py` initializes hardware, shares, queues, and tasks
- `cotask.py` implements cooperative task scheduling
- `task_share.py` implements inter-task shared data and queues
- `left_drive_system.py` and `right_drive_system.py` handle each motor/encoder pair
- `ClosedLoop.py` performs wheel PI control and line-following logic
- `task_line_sensor.py` reads the line sensor
- `linesensor.py` performs calibration, normalization, and centroid calculation
- `task_bump.py` handles bump detection
- `task_user.py` provides a serial tuning and diagnostics interface

## Control Strategy

### Closed-Loop Wheel Control
The robot uses PI control for left and right wheel velocity. Each wheel has a setpoint and measured velocity, and the controller computes motor effort from the wheel-speed error.

### Line Following
The line-following controller computes a centroid from the 8-sensor array, with the center target near sensor index 4. The line-following error is the difference between the measured centroid and the desired center.

The controller applies **PI control to the line-following error**, which allows smooth and continuous steering adjustments rather than abrupt corrections. A steering correction is applied by adjusting the left and right wheel velocities around a forward base speed.

This means:
- if the line appears to the left, the robot steers left,
- if the line appears to the right, the robot steers right,
- and if the line is centered, both wheels run at nearly equal speed.

## Lost-Line and Recovery Behavior
If the line sensor loses the line for several consecutive cycles, the robot begins a scripted recovery sequence. This sequence includes:
1. moving forward to square up,
2. moving farther into the garage area,
3. making a 90-degree right turn,
4. driving forward with encoder-based straightening,
5. then rotating left until the line is found again.

## Important Math and Parameters
Some important geometric and sensing values used in the project are:

- Wheel radius: **35 mm**
- Wheel diameter: **70 mm**
- Track width: **141 mm**
- Encoder resolution: **about 1430 to 1437 counts/rev**

A critical parameter in our implementation is the **conversion between encoder ticks and physical distance**. The relationship between encoder counts and linear distance is hard-coded in the software. This conversion factor determines how encoder measurements are translated into real-world units such as mm/s.

Because this value is fixed in the code, accuracy in the encoder ticks-per-unit-length conversion is essential. Any error in this parameter directly affects velocity estimation, distance tracking, and overall control performance.

## Video Demonstration
**Embedded video or link:**  
[PASTE YOUR VIDEO LINK HERE]
