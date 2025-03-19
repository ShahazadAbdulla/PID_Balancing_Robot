#include <Servo.h>

#define MAX_SIGNAL 2000
#define MIN_SIGNAL 1000
#define MOTOR_PIN 9

Servo motor;

void setup() {
  Serial.begin(9600);
  Serial.println("ELECTRONOOBS ESC Calibration...");
  motor.attach(MOTOR_PIN);

  // Inform the user about the calibration procedure
  Serial.println("Power on and press any key after 2 seconds to start.");

  motor.writeMicroseconds(MAX_SIGNAL);  // Max output for ESC calibration
  while (!Serial.available());  // Wait for user input
  Serial.read();  // Clear the input buffer

  motor.writeMicroseconds(MIN_SIGNAL);  // Min output to complete calibration
  Serial.println("ESC Calibration Complete.");
  Serial.println("Enter a value between 1000 and 2000 for motor speed.");
}

void loop() {
  if (Serial.available() > 0) {
    int DELAY = Serial.parseInt();
    if (DELAY >= 1000 && DELAY <= 2000) {
      motor.writeMicroseconds(DELAY);
      float SPEED = (DELAY - 1000) / 10.0;
      Serial.print("Motor speed: ");
      Serial.print(SPEED);
      Serial.println("%");
    }
  }
}
