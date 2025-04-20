#include <ESP32Servo.h>

Servo ESC1;  // Servo object for ESC on pin 9
Servo ESC2;  // Servo object for ESC on pin 10
Servo ESC3;  // Servo object for ESC on pin 11
Servo ESC4;  // Servo object for ESC on pin 12
int ESCfreq = 500;
int currentSpeed = 1000;  // Stores the last speed value to be sent continuously

void setup() {
  Serial.begin(115200);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  delay(1000);
  ESC1.attach(19, 1000, 2000);
  delay(1000);
  ESC1.setPeriodHertz(ESCfreq);
  delay(100);
  ESC2.attach(18, 1000, 2000);
  delay(1000);
  ESC2.setPeriodHertz(ESCfreq);
  delay(100);
  ESC3.attach(5, 1000, 2000);
  delay(1000);
  ESC3.setPeriodHertz(ESCfreq);
  delay(100);
  ESC4.attach(4, 1000, 2000);  // Attach ESC1 to pin 9 with pulse range from 1000 to 2000 µs
  delay(1000);
  ESC4.setPeriodHertz(ESCfreq);
  delay(100);

  // ESC1.writeMicroseconds(1000);
  // ESC2.writeMicroseconds(1000);
  // ESC3.writeMicroseconds(1000);
  // ESC4.writeMicroseconds(1000);  // Initialize serial communication at 9600 baud
  Serial.println("Enter a value (0-1023):");
}

void loop() {
  // Check if a complete line of input is available from the Serial Monitor
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');  // Read until newline
    int newValue = input.toInt();                 // Convert the input to an integer

    // If the conversion returns 0 and the input isn't "0", it might be an invalid number.
    // You may want to add additional error checking here.
    if (newValue != 0 || input == "0") {
      // Map the input value (0-1023) to the expected ESC range (0-180)
      currentSpeed = newValue;
      // currentSpeed = map(newValue, 1000, 2000, 0, 100);
      Serial.print("Set speed to: ");
      Serial.println(currentSpeed);
      ESC1.write(currentSpeed);  // Send speed to ESC1
      ESC2.write(currentSpeed);  // Send speed to ESC1
      ESC3.write(currentSpeed);  // Send speed to ESC1
      ESC4.write(currentSpeed);  // Send speed to ESC1
    }
  }

  // Continuously send the current speed value to each ESC
}
