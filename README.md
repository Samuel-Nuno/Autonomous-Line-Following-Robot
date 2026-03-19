# Autonomous-Line-Following-Robot

# Romi Time-Trials Final Project
**Team Members:** Samuel Nuno, Bruce Berg

## Project Summary
This project documents our ME 405 Romi final project, in which we developed a differential-drive robot capable of autonomous line following with closed-loop wheel control, line position estimation, and obstacle-triggered recovery behavior. The goal of the project was to create a robot that could reliably follow the printed course at speed while using organized, reproducible software and clear system documentation.

## Project Objective
Our robot was designed to complete the Romi line-following time-trial by combining encoder-based wheel feedback, analog line sensing, and task-based control software. The system was structured to prioritize repeatable behavior and robustness rather than relying on open-loop tuning alone.

## Mechanical and Electrical Design
The platform is based on the Pololu Romi differential-drive chassis. The robot uses two independently driven wheels, each with its own motor and encoder feedback. In our preliminary design work, we identified several key chassis parameters that informed our control and sensing decisions, including a wheel radius of 35 mm and a track width of 141 mm.

The electrical system includes:
- Two DC drive motors
- Wheel encoders for each side
- An STM32 microcontroller running MicroPython
- A Pololu QTR-8A analog reflectance sensor array for line following
- A bump switch for collision/contact detection
- Motor driver outputs using PWM and direction pins

### Motor and Encoder Interface
Each motor is controlled by a PWM signal and a direction pin. Encoder feedback is used to estimate wheel speed and wheel position continuously.

## Sensors Used
### 1. Wheel Encoders
We used one quadrature encoder per wheel to estimate wheel position and velocity. In software, encoder counts are converted into wheel speed in mm/s. Our encoder model uses a wheel diameter of 70 mm and an effective encoder resolution of about 1430 counts per wheel revolution.

### 2. Line Sensor Array
We selected an 8-channel analog Pololu QTR-8A reflectance sensor array. Our line-sensor design memo concluded that an analog array was the best choice because it preserves continuous position information, unlike digital threshold-only sensing. We also concluded that 5 to 8 channels gives enough resolution for smooth steering corrections while remaining simple to integrate.

The line sensor is calibrated on both white background and black line surfaces. After calibration, the software normalizes each sensor channel and computes a centroid value from 0 to 7 to estimate line position.

### 3. Bump Sensor
A bump switch is used as a contact sensor. It provides a simple and robust way to detect collision with the wall or barrier and trigger a recovery routine.

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
- `task_bump.py` debounces the bump switch
- `task_user.py` provides a serial tuning and diagnostics interface

## Control Strategy
### Closed-Loop Wheel Control
The robot uses PI control for left and right wheel velocity. Each wheel has a setpoint and measured velocity, and the controller computes motor effort from the wheel-speed error. The controller also includes logic to:
- reset the integral term when setpoints change sharply,
- saturate the output effort,
- and avoid unnecessary integration near zero-speed operation.

### Line Following
The line-following controller computes a centroid from the 8-sensor array, with the center target near sensor index 4. The line-following error is the difference between the measured centroid and the desired center. A steering correction is then applied by changing the left and right wheel setpoints around a forward base speed.

This means:
- if the line appears to the left, the robot steers left,
- if the line appears to the right, the robot steers right,
- and if the line is centered, both wheels run at nearly equal speed.

The controller also includes:
- a deadband around the center,
- steering slew limiting,
- centroid-loss detection,
- and a fallback routine if the line is lost.

## Lost-Line and Recovery Behavior
One of the most distinctive parts of our final implementation is the recovery or “garage” script inside the closed-loop controller. If the line sensor loses the line for several consecutive cycles, the robot begins a scripted recovery sequence. This sequence includes:
1. moving forward to square up,
2. moving farther into the garage area,
3. making a 90-degree right turn,
4. driving forward with encoder-based straightening,
5. then rotating left until the line is found again.

This behavior allows the robot to recover from course features or temporary loss of the line rather than simply stopping.

## Important Math and Parameters
Some important geometric and sensing values used in the project are:

- Wheel radius: **35 mm**
- Wheel diameter: **70 mm**
- Track width: **141 mm**
- Encoder resolution: **about 1430 to 1437 counts/rev**
- Encoder distance scale: **about 0.153 mm/count**

Wheel velocity is computed from encoder delta counts and elapsed time, then converted from counts/s into mm/s.

## Results and Performance
Our final system successfully integrated:
- closed-loop motor control,
- line tracking with centroid estimation,
- line-loss detection,
- contact detection using a bump switch,
- and a multitask software structure that kept sensing, control, and user interaction modular.

The line-following implementation was strengthened by analog sensing, calibration, and centroid-based steering instead of simple threshold logic. This gave us smoother and more informative feedback for control.

## Challenges and Lessons Learned
A major challenge in this project was balancing speed with reliability. It was not enough for the robot to detect the line; the robot also needed to respond quickly without overcorrecting. Sensor calibration, steering sensitivity, and wheel control tuning all affected that balance.

Another challenge was handling edge cases such as:
- startup with no valid line reading,
- line loss,
- large steering transients,
- and interaction between sensing and motion control.

Using task-based software and shared variables made the project easier to organize and debug. In particular, separating the line sensor, bump sensor, drive systems, and control logic into dedicated modules made it much easier to test each subsystem independently.

## Video Demonstration
**Embedded video or link:**  
[PASTE YOUR YOUTUBE OR DRIVE VIDEO LINK HERE]

## Photos
### Robot Overview
![Robot photo](PASTE_IMAGE_LINK_HERE)

### Sensor Mount / Front View
![Sensor mount photo](PASTE_IMAGE_LINK_HERE)

### Wiring / Electronics
![Wiring photo](PASTE_IMAGE_LINK_HERE)

## Repository
**GitHub Repository:**  
[PASTE YOUR GITHUB REPOSITORY LINK HERE]

## Source Files
Key source files included in the repository:
- `main.py`
- `ClosedLoop.py`
- `left_drive_system.py`
- `right_drive_system.py`
- `encoder.py`
- `motor.py`
- `task_line_sensor.py`
- `linesensor.py`
- `task_bump.py`
- `task_user.py`
- `cotask.py`
- `task_share.py`
- `IMU_Driver.py`

## Supporting Documents
Include links to:
- Preliminary Romi memo
- Line sensor acquisition memo
- Wiring diagram
- Mechanical drawings or sensor mount files
- Video demo
- Any plots or tuning results

### Suggested document links
- [Preliminary Romi Memo](PASTE_LINK_HERE)
- [Line Sensor Memo](PASTE_LINK_HERE)
- [Wiring Diagram](PASTE_LINK_HERE)
- [CAD / Mount Files](PASTE_LINK_HERE)
- [Video Demo](PASTE_LINK_HERE)
