# PID Balancing Robot

This project is a balancing robot that uses PID control to maintain its balance. The robot uses an MPU-6050 IMU (accelerometer and gyroscope) to sense its orientation and adjusts its motors to stay balanced. The robot uses ESCs (Electronic Speed Controllers) to control the motors' speed.

![WhatsApp Image 2025-03-19 at 21 06 54](https://github.com/user-attachments/assets/92982983-194d-4eb7-b2e6-e08bb1a2819a)
![WhatsApp Image 2025-03-19 at 21 06 18](https://github.com/user-attachments/assets/e9715bf4-c6bf-4ef2-8073-34beb57c73c9)



## Features

- **PID Balance Control**: The robot uses a PID controller to adjust the speed of the motors and keep itself upright.
- **ESC Calibration**: The ESC calibration sketch ensures that the ESCs are correctly calibrated to control the motors.
- **Motor Control**: PWM signals control the motors, providing speed control based on the PID output.

## Hardware Requirements

- **MPU-6050 IMU**: For sensing orientation (accelerometer and gyroscope).
- **ESCs**: For controlling the motors.
- **Brushless DC Motors (BLDC)**: For propulsion and balancing.
- **Arduino Board**: For processing sensor data and controlling the motors.
- **Servo Motors**: For propelling the robot.

## Software Requirements

- **Arduino IDE**: To upload the code to the Arduino board.
- **Servo Library**: For controlling servo motors.
- **Wire Library**: For I2C communication with the MPU-6050.

## Setup and Usage

1. **ESC Calibration**:
    - Upload the `ESC_calibrate.ino` file to the Arduino.
    - Turn on the power, wait 2 seconds, and press any key to start calibration.
    - After calibration, enter a value between 1000 and 2000 to control the motor speed.

2. **Balance Control**:
    - Upload the `PID_balance_arduino.ino` file to the Arduino.
    - The robot will use PID control to maintain balance based on the data from the MPU-6050.

## PID Control Parameters

- **Proportional (Kp)**: Controls the proportional gain for the error.
- **Integral (Ki)**: Controls the integral gain for accumulated error over time.
- **Derivative (Kd)**: Controls the derivative gain based on the rate of change of error.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgements

- Thanks to the creators of the Arduino platform and libraries used in this project.
- MPU-6050 documentation for detailed sensor data.
