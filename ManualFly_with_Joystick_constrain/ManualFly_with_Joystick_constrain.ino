#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <LittleFS.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <PID_v1.h>  // Include PID Library

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define INTERRUPT_PIN 25

// PID Variables for Yaw, Pitch, and Roll
double setpointYaw = 0.0, setpointPitch = 0.0, setpointRoll = 0.0;
double inputYaw, inputPitch, inputRoll;
double outputYaw, outputPitch, outputRoll;
const int trigPin = 5;  
const int echoPin = 18;

  // PID tuning parameters (adjust as needed)
  double Kp = 1.5,
         Ki = 0.05, Kd = 0.1;

// Create PID objects
PID pidYaw(&inputYaw, &outputYaw, &setpointYaw, Kp, Ki, Kd, DIRECT);
PID pidPitch(&inputPitch, &outputPitch, &setpointPitch, Kp, Ki, Kd, DIRECT);
PID pidRoll(&inputRoll, &outputRoll, &setpointRoll, Kp, Ki, Kd, DIRECT);

// MPU variables
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

// Interrupt detection
volatile bool mpuInterrupt = false;


// WiFi Access Point
const char *ssid = "QuadcopterControl";
const char *password = NULL;

const int webPort = 80;
WebServer server(webPort);
DNSServer dnsServer;
bool kill_switch = false;
// ESC & Motors
Servo mot1, mot2, mot3, mot4;
const int mot1_pin = 13;
const int mot2_pin = 12;
const int mot3_pin = 14;
const int mot4_pin = 27;
int motor1, motor2, motor3, motor4;
const int minPWM = 1000, maxPWM = 2000, basePWM = 1500;
// Structure to hold joystick values
struct JoystickData {
  int roll;
  int pitch;
  int throttle;
  int yaw;
};

JoystickData joystick = { 500, 500, 1500, 500 };  // Neutral Position

// Motor Speeds
int m1_speed = 1000, m2_speed = 1000, m3_speed = 1000, m4_speed = 1000;
void dmpDataReady() {
  mpuInterrupt = true;
}
// ESC Calibration
void escCalibration() {
  Serial.println("Starting ESC Calibration...");
  mot1.writeMicroseconds(2000);
  mot2.writeMicroseconds(2000);
  mot3.writeMicroseconds(2000);
  mot4.writeMicroseconds(2000);
  delay(7000);
  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
  Serial.println("ESC Calibration Complete!");
  delay(7000);
}

void handleJoystick() {
  if (server.args() == 4) {
    joystick.roll = server.arg("roll").toInt();
    joystick.pitch = server.arg("pitch").toInt();
    joystick.throttle = server.arg("throttle").toInt();
    joystick.yaw = server.arg("yaw").toInt();

    // ✅ Fix: Keep throttle within the correct range without incorrect mapping
    joystick.throttle = constrain(joystick.throttle, 1000, 2000);

    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Invalid Request");
  }
}
// Serve web interface files
void handleFileServer() {
  String path = server.uri();
  if (path == "/") path = "/index.html";

  String contentType = "text/html";
  if (LittleFS.exists(path)) {
    File file = LittleFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
  } else {
    server.send(404, "text/plain", "NOT FOUND");
  }
}

// Send motor speeds to web UI
void handleMotorSpeed() {
  String json = "{";
  json += "\"m1\":" + String(m1_speed) + ",";
  json += "\"m2\":" + String(m2_speed) + ",";
  json += "\"m3\":" + String(m3_speed) + ",";
  json += "\"m4\":" + String(m4_speed) + "}";
  server.send(200, "application/json", json);
}

void updateMotors() {
  if (kill_switch) {
    return;
  }
  int baseThrottle = joystick.throttle;
  baseThrottle = constrain(baseThrottle, 1000, 2000);

  int rollEffect = map(joystick.roll, 0, 1000, -300, 300);
  int pitchEffect = map(joystick.pitch, 0, 1000, -300, 300);
  int yawEffect = map(joystick.yaw, 0, 1000, -300, 300);

  // ✅ Set all motors to base throttle initially
  m1_speed = baseThrottle;
  m2_speed = baseThrottle;
  m3_speed = baseThrottle;
  m4_speed = baseThrottle;
  if (pitchEffect > 0) {
    // **Pitch Down:** Increase rear motors (M3 & M4)
    m1_speed = baseThrottle;
    m2_speed = baseThrottle;
    m3_speed += pitchEffect;
    m4_speed += pitchEffect;
  } else if (pitchEffect < 0) {
    // **Pitch Up:** Increase front motors (M1 & M2)
    m3_speed = baseThrottle;
    m4_speed = baseThrottle;
    m1_speed -= pitchEffect;
    m2_speed -= pitchEffect;
  } else if (rollEffect > 0) {
    // **Roll Right:** Increase right motors (M2 & M4)
    m2_speed = baseThrottle;
    m3_speed = baseThrottle;
    m1_speed += rollEffect;
    m4_speed += rollEffect;
  } else if (rollEffect < 0) {
    // **Roll Left:** Increase left motors (M1 & M3)
    m1_speed = baseThrottle;
    m4_speed = baseThrottle;
    m2_speed -= rollEffect;
    m3_speed -= rollEffect;
  }

  // ✅ Yaw Control
  else if (yawEffect > 0) {
    // **Yaw Right:** Increase M1 & M3, decrease M2 & M4
    m1_speed += yawEffect;
    m3_speed += yawEffect;
    m2_speed -= yawEffect;
    m4_speed -= yawEffect;
  } else if (yawEffect < 0) {
    // **Yaw Left:** Increase M2 & M4, decrease M1 & M3
    m2_speed += -yawEffect;
    m4_speed += -yawEffect;
    m1_speed -= -yawEffect;
    m3_speed -= -yawEffect;
  }

  // ✅ Read Yaw, Pitch, and Roll as input for PID
  inputYaw = ypr[0] * 180 / M_PI;
  inputPitch = ypr[1] * 180 / M_PI;
  inputRoll = ypr[2] * 180 / M_PI;

  // Compute PID outputs
  pidYaw.Compute();
  pidPitch.Compute();
  pidRoll.Compute();

  motor1 = constrain(basePWM + outputPitch - outputRoll + outputYaw, minPWM, maxPWM);
  motor2 = constrain(basePWM + outputPitch + outputRoll - outputYaw, minPWM, maxPWM);
  motor3 = constrain(basePWM - outputPitch - outputRoll - outputYaw, minPWM, maxPWM);
  motor4 = constrain(basePWM - outputPitch + outputRoll + outputYaw, minPWM, maxPWM);

  Serial.print("Motor1: ");
  Serial.print(motor1);
  Serial.print(" | Motor2: ");
  Serial.print(motor2);
  Serial.print(" | Motor3: ");
  Serial.print(motor3);
  Serial.print(" | Motor4: ");
  Serial.print(motor4);


  // ✅ Ensure motor speeds stay within ESC range (1000-2000)
  m1_speed = constrain(m1_speed, 900, 1700);
  m2_speed = constrain(m2_speed, 900, 1700);
  m3_speed = constrain(m3_speed, 900, 1700);
  m4_speed = constrain(m4_speed, 900, 1700);

  // ✅ Send values to motors
  mot1.writeMicroseconds(m1_speed);
  mot2.writeMicroseconds(m2_speed);
  mot3.writeMicroseconds(m3_speed);
  mot4.writeMicroseconds(m4_speed);

  Serial.print("  M1: ");
  Serial.print(m1_speed);
  Serial.print(" | M2: ");
  Serial.print(m2_speed);
  Serial.print(" | M3: ");
  Serial.print(m3_speed);
  Serial.print(" | M4: ");
  Serial.print(m4_speed);
}

void handleKillSwitch() {
  m1_speed = 0;
  m2_speed = 0;
  m3_speed = 0;
  m4_speed = 0;
  kill_switch = true;
  mot1.writeMicroseconds(0);
  mot2.writeMicroseconds(0);
  mot3.writeMicroseconds(0);
  mot4.writeMicroseconds(0);

  Serial.println("Kill switch activated! Motors stopped.");
  server.send(200, "text/plain", "Motors Stopped");
}

// Setup function
void setup() {
  Serial.begin(115200);
  escCalibration();

  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS!");
    while (true)
      ;
  }

  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  dnsServer.start(53, "*", WiFi.softAPIP());
  server.on("/joystick", HTTP_GET, handleJoystick);
  server.on("/motorspeed", HTTP_GET, handleMotorSpeed);
  server.on("/kill", HTTP_GET, handleKillSwitch);
  // New endpoint
  server.on("/", handleFileServer);
  server.begin();
  Serial.println("Web Server Started");
  Wire.begin();
  Wire.setClock(100000);

  pinMode(INTERRUPT_PIN, INPUT_PULLUP);

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(8);
  mpu.setZGyroOffset(21);
  mpu.setXAccelOffset(1150);
  mpu.setYAccelOffset(-50);
  mpu.setZAccelOffset(1060);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    Serial.println();
    mpu.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // Initialize PIDs
  pidYaw.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidRoll.SetMode(AUTOMATIC);

  pidYaw.SetOutputLimits(-255, 255);
  pidPitch.SetOutputLimits(-255, 255);
  pidRoll.SetOutputLimits(-255, 255);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  mot1.attach(mot1_pin, 700, 1800);
  mot2.attach(mot2_pin, 700, 1800);
  mot3.attach(mot3_pin, 700, 1800);
  mot4.attach(mot4_pin, 700, 1800);
}

// Loop function
void loop() {
  server.handleClient();
  dnsServer.processNextRequest();
  updateMotors();
  // Serial.println(kill_switch);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  // Trigger the sensor by sending a 10us HIGH pulse:
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the length of the echo pulse:
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters:
  // Sound speed ~343 m/s -> 29.1 us per centimeter round trip (~58 us per cm for the pulse time).
  float distance = duration / 58.0;

  Serial.print("   Distance: ");
  Serial.print(distance);
  Serial.println(" cm");


  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  }
  // delay(100);
}
