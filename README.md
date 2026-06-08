# Relay-Bot
This project highlights a robust embedded finite-state machine (FSM) designed to guide a differential-drive mobile manipulator through a complex, event-driven tracking, payload acquisition, and delivery routine.

## Feature Pipeline & Sensor Integration
### Event-Triggered Initialization (Sonar Processing)
The robot begins in a low-power, non-blocking standby loop. An ultrasonic sonar sensor continuously polls proximity metrics. The state machine triggers navigation if a physical flag is raised; otherwise, it handles edge cases through an automated time-delayed fallback structure.
### Closed-Loop Navigation (Thresholding Logic)
Line-tracking arrays continuously scan the surface. By utilizing specialized calibration thresholding algorithms, the firmware sharply differentiates between high-contrast black and white environments. This allows the robot to handle tracking adjustments dynamically, managing sweeping curves and sharp corners flawlessly.
### Manipulator Actuation (Servo Motor Optimization)
Once the object zone is verified by proximity feedback, the firmware executes an synchronized pulse-width modulation (PWM) sequence across a bank of servo motors. The logic calculates target angular profiles to approach and grasp a cup while moving.
### Terminal Target Identification & Handoff
Upon detecting a solid terminal black square boundary, the system exits the tracking loop, decelerates, opens the servo-driven gripper to ground the payload safely, and prompts a clean microcontroller halt sequence.

## Tech Stack & Keywords
- Languages: C++
- Libraries: Servo.h, Adafruit_NeoPixel.h
