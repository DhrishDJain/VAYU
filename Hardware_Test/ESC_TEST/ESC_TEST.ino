#include <ESP32Servo.h>


Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;
bool calibrated = false;
const int mot1_pin = 19;
const int mot2_pin = 18;
const int mot3_pin = 5;  //14 for perf board
const int mot4_pin = 4;
void setup() {
  Serial.begin(115200);
  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);
  mot3.attach(mot3_pin, 1000, 2000);
  mot4.attach(mot4_pin, 1000, 2000);
  Serial.println("=== ESC CALIBRATION MODE ===");
  Serial.println("Disconnect battery. Then press any key in Serial Monitor...");

  // while (!Serial.available()) {
  //   // Wait for user input to start calibration
  // }

  // Serial.read();  // Clear the input

  mot1.writeMicroseconds(2000);
  mot2.writeMicroseconds(2000);
  mot3.writeMicroseconds(2000);
  mot4.writeMicroseconds(2000);
  Serial.println("Sending MAX signal (2000)... wait for beeps...");
  delay(3000);  // Wait for beeps

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
  Serial.println("Sending MIN signal (1000)... wait for confirmation tones...");
  delay(3000);  // Wait for arming tones

  Serial.println("Calibration done. ESC is now armed.");
  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);  // Keep ESC armed at min throttle
  calibrated = true;
}

void loop() {
  // You can test motor control here after calibration
  if (calibrated) {
    Serial.println("Increasing throttle...");
    for (int speed = 1000; speed <= 2000; speed += 100) {
      mot1.writeMicroseconds(speed);
      mot2.writeMicroseconds(speed);
      mot3.writeMicroseconds(speed);
      mot4.writeMicroseconds(speed);
      Serial.print("Throttle: ");
      Serial.println(speed);
      delay(1000);
    }

    Serial.println("Decreasing throttle...");
    for (int speed = 2000; speed >= 1000; speed -= 100) {
      mot1.writeMicroseconds(speed);
      mot2.writeMicroseconds(speed);
      mot3.writeMicroseconds(speed);
      mot4.writeMicroseconds(speed);
      Serial.print("Throttle: ");
      Serial.println(speed);
      delay(1000);
    }

    Serial.println("Test complete.");
    calibrated = false;
    mot1.writeMicroseconds(1000);
    mot2.writeMicroseconds(1000);
    mot3.writeMicroseconds(1000);
    mot4.writeMicroseconds(1000);  // Set throttle to min
  }

  delay(5000);  // Wait before next loop
}