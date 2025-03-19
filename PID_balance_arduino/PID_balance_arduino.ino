#include <Wire.h>
#include <Servo.h>

Servo right_prop, left_prop;
int16_t Acc_rawX, Acc_rawY, Acc_rawZ, Gyr_rawX, Gyr_rawY, Gyr_rawZ;
float Acceleration_angle[2], Gyro_angle[2], Total_angle[2];
float elapsedTime, time, timePrev, error, previous_error;
float pid_p, pid_i, pid_d, throttle = 1300, desired_angle = 0;

// PID constants
double kp = 3.55, ki = 0.005, kd = 2.05;

void setup() {
  Wire.begin();
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);  // Wake up the MPU6050
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(250000);

  right_prop.attach(3);
  left_prop.attach(5);

  // Initialize motors with min PWM
  left_prop.writeMicroseconds(1000);
  right_prop.writeMicroseconds(1000);
  delay(7000);  // Wait for ESC startup
}

void loop() {
  timePrev = time;
  time = millis();
  elapsedTime = (time - timePrev) / 1000.0;  // Time in seconds
  
  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Starting address for accelerometer data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);
  Acc_rawX = Wire.read() << 8 | Wire.read();
  Acc_rawY = Wire.read() << 8 | Wire.read();
  Acc_rawZ = Wire.read() << 8 | Wire.read();

  // Convert to acceleration in g
  Acceleration_angle[0] = atan(Acc_rawY / 16384.0 / sqrt(pow(Acc_rawX / 16384.0, 2) + pow(Acc_rawZ / 16384.0, 2))) * (180 / PI);
  Acceleration_angle[1] = atan(-Acc_rawX / 16384.0 / sqrt(pow(Acc_rawY / 16384.0, 2) + pow(Acc_rawZ / 16384.0, 2))) * (180 / PI);

  // Read gyroscope data
  Wire.beginTransmission(0x68);
  Wire.write(0x43);  // Starting address for gyroscope data
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 4, true);
  Gyr_rawX = Wire.read() << 8 | Wire.read();
  Gyr_rawY = Wire.read() << 8 | Wire.read();

  // Convert to degrees/second
  Gyro_angle[0] = Gyr_rawX / 131.0;
  Gyro_angle[1] = Gyr_rawY / 131.0;

  // Combine accelerometer and gyroscope data with complementary filter
  Total_angle[0] = 0.98 * (Total_angle[0] + Gyro_angle[0] * elapsedTime) + 0.02 * Acceleration_angle[0];
  Total_angle[1] = 0.98 * (Total_angle[1] + Gyro_angle[1] * elapsedTime) + 0.02 * Acceleration_angle[1];

  // PID control to maintain balance
  error = Total_angle[1] - desired_angle;
  pid_p = kp * error;
  
  // Integral part of PID (only activate for small errors)
  if (abs(error) < 3) {
    pid_i += ki * error;
  }

  // Derivative part of PID
  pid_d = kd * ((error - previous_error) / elapsedTime);

  // Calculate final PID value
  float PID = pid_p + pid_i + pid_d;
  PID = constrain(PID, -1000, 1000);

  // Calculate motor PWM values
  float pwmLeft = throttle + PID;
  float pwmRight = throttle - PID;

  // Ensure PWM values are within valid range (1000 - 2000 us)
  pwmLeft = constrain(pwmLeft, 1000, 2000);
  pwmRight = constrain(pwmRight, 1000, 2000);

  // Apply PWM to motors
  left_prop.writeMicroseconds(pwmLeft);
  right_prop.writeMicroseconds(pwmRight);

  previous_error = error;  // Store error for the next loop
}
