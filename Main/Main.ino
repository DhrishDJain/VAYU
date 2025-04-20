#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <ESPAsyncWebServer.h>  //V3.6.0
#include <WebSocketsServer.h>
// #include <AsyncTCP.h> //V1.1.4
#include <LittleFS.h>
#include <ArduinoJson.h>  //V7.3.0
#include "Wire.h"
MPU6050 mpu;
#define INTERRUPT_PIN 23
bool Kill = false;

// PID tuning parameters (adjust as needed)
double Kp = 0.5,
       Ki = 0.7, Kd = 0.001;

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

AsyncWebServer server(80);
WebSocketsServer websockets(81);
// ESC & Motors
Servo M1, M2, M3, M4;
const int M1_pin = 19;
const int M2_pin = 18;
const int M3_pin = 5;
const int M4_pin = 4;
int motor1, motor2, motor3, motor4;
const int minPWM = 1000, maxPWM = 2000;
int m1_speed = 0, m2_speed = 0, m3_speed = 1000, m4_speed = 0;
float t = 0.004;
float PAngleRoll = 2;
float IAngleRoll = 0.5;
float DAngleRoll = 0.007;
float PAnglePitch = PAngleRoll;
float IAnglePitch = IAngleRoll;
float DAnglePitch = DAngleRoll;

float PRateRoll = 0.5;
float IRateRoll = 0.7;
float DRateRoll = 0.001;
float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

volatile float MotorInput1, MotorInput2, MotorInput3, MotorInput4;
// Structure to hold joystick values
struct JoystickData {
  int roll;
  int pitch;
  int throttle;
  int yaw;
};

JoystickData joystick = { 0, 0, 1000, 0 };  // Neutral Position
// PID Output Var
volatile float PtermRoll;
volatile float ItermRoll;
volatile float DtermRoll;
volatile float PIDOutputRoll;
volatile float PtermPitch;
volatile float ItermPitch;
volatile float DtermPitch;
volatile float PIDOutputPitch;
volatile float PtermYaw;
volatile float ItermYaw;
volatile float DtermYaw;
volatile float PIDOutputYaw;
// ===========================  Web Functions    =============================
void sendtxt(String txt) {
  Serial.println(txt);
  websockets.sendTXT(0, txt);
}

void webSocketEvent(uint8_t clientId, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_CONNECTED) {
    IPAddress ip = websockets.remoteIP(clientId);
    Serial.printf("[%u] Connected from %d.%d.%d.%d\n", clientId, ip[0], ip[1], ip[2], ip[3]);
  } else if (type == WStype_TEXT) {
    String receivedPayload((char *)payload, length);
    Serial.printf("Received from client [%u]: %s\n", clientId, receivedPayload.c_str());
    DynamicJsonDocument message(1024);
    DeserializationError error = deserializeJson(message, receivedPayload);

    if (error) {
      Serial.print(F("deserializeJson() failed: "));
      Serial.println(error.f_str());
      return;
    }
    String path = message["path"];
    if (message["action"] == "KILL") {
      Kill = true;
    }
    if (message["action"] == "JoystickData") {
      joystick.throttle = message["throttle"];
      joystick.throttle = constrain(joystick.throttle, 1000, 2000);
      joystick.pitch = message["pitch"];
      joystick.roll = message["roll"];
      joystick.yaw = message["yaw"];
    }
  }
}
void dmpDataReady() {
  mpuInterrupt = true;
}

// Send motor speeds to web UI
void handleMotorSpeed() {
  String json = "{";
  json += "\"m1\":" + String(m1_speed) + ",";
  json += "\"m2\":" + String(m2_speed) + ",";
  json += "\"m3\":" + String(m3_speed) + ",";
  json += "\"m4\":" + String(m4_speed) + "}";
  sendtxt(json);
}

// void updateMotors() {

//   int baseThrottle = joystick.throttle;
//   baseThrottle = constrain(baseThrottle, 1000, 2000);

//   int rollEffect = map(joystick.roll, 0, 1000, -300, 300);
//   int pitchEffect = map(joystick.pitch, 0, 1000, -300, 300);
//   int yawEffect = map(joystick.yaw, 0, 1000, -300, 300);

//   // ✅ Set all motors to base throttle initially
//   m1_speed = baseThrottle;
//   m2_speed = baseThrottle;
//   m3_speed = baseThrottle;
//   m4_speed = baseThrottle;
//   if (pitchEffect > 0) {
//     // **Pitch Down:** Increase rear motors (M3 & M4)
//     m1_speed = baseThrottle;
//     m2_speed = baseThrottle;
//     m3_speed += pitchEffect;
//     m4_speed += pitchEffect;
//   } else if (pitchEffect < 0) {
//     // **Pitch Up:** Increase front motors (M1 & M2)
//     m3_speed = baseThrottle;
//     m4_speed = baseThrottle;
//     m1_speed -= pitchEffect;
//     m2_speed -= pitchEffect;
//   } else if (rollEffect > 0) {
//     // **Roll Right:** Increase right motors (M2 & M4)
//     m2_speed = baseThrottle;
//     m3_speed = baseThrottle;
//     m1_speed += rollEffect;
//     m4_speed += rollEffect;
//   } else if (rollEffect < 0) {
//     // **Roll Left:** Increase left motors (M1 & M3)
//     m1_speed = baseThrottle;
//     m4_speed = baseThrottle;
//     m2_speed -= rollEffect;
//     m3_speed -= rollEffect;
//   }

//   // ✅ Yaw Control
//   else if (yawEffect > 0) {
//     // **Yaw Right:** Increase M1 & M3, decrease M2 & M4
//     m1_speed += yawEffect;
//     m3_speed += yawEffect;
//     m2_speed -= yawEffect;
//     m4_speed -= yawEffect;
//   } else if (yawEffect < 0) {
//     // **Yaw Left:** Increase M2 & M4, decrease M1 & M3
//     m2_speed += -yawEffect;
//     m4_speed += -yawEffect;
//     m1_speed -= -yawEffect;
//     m3_speed -= -yawEffect;
//   }

//   // ✅ Read Yaw, Pitch, and Roll as input for PID
//   inputYaw = ypr[0] * 180 / M_PI;
//   inputPitch = ypr[1] * 180 / M_PI;
//   inputRoll = ypr[2] * 180 / M_PI;

//   motor1 = constrain(joystick.throttle + outputPitch - outputRoll + outputYaw, minPWM, maxPWM);
//   motor2 = constrain(joystick.throttle + outputPitch + outputRoll - outputYaw, minPWM, maxPWM);
//   motor3 = constrain(joystick.throttle - outputPitch - outputRoll - outputYaw, minPWM, maxPWM);
//   motor4 = constrain(joystick.throttle - outputPitch + outputRoll + outputYaw, minPWM, maxPWM);

//   Serial.print("PID_M1: ");
//   Serial.print(motor1);
//   Serial.print(" | PID_M2: ");
//   Serial.print(motor2);
//   Serial.print(" | PID_M3: ");
//   Serial.print(motor3);
//   Serial.print(" | PID_M4: ");
//   Serial.print(motor4);

//   // ✅ Ensure motor speeds stay within ESC range (1000-2000)
//   m1_speed = constrain(m1_speed, 900, 1700);
//   m2_speed = constrain(m2_speed, 900, 1700);
//   m3_speed = constrain(m3_speed, 900, 1700);
//   m4_speed = constrain(m4_speed, 900, 1700);

//   // ✅ Send values to motors
//   M1.writeMicroseconds(m1_speed);
//   M2.writeMicroseconds(m2_speed);
//   M3.writeMicroseconds(m3_speed);
//   M4.writeMicroseconds(m4_speed);

//   Serial.print("  M1: ");
//   Serial.print(m1_speed);
//   Serial.print(" | M2: ");
//   Serial.print(m2_speed);
//   Serial.print(" | M3: ");
//   Serial.print(m3_speed);
//   Serial.print(" | M4: ");
//   Serial.print(m4_speed);
// }

// Setup function
void setup() {
  Serial.begin(115200);

  if (!LittleFS.begin()) {
    Serial.println("Failed to mount LittleFS!");
    while (true)
      ;
  }

  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(LittleFS, "/index.html", "text/html", false);
  });
  server.serveStatic("/", LittleFS, "/");
  server.begin();
  Serial.println("Web Server Started");

  // MPU Setup
  Wire.begin();
  Wire.setClock(100000);
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
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

  // ESC Calibration
  M1.attach(M1_pin, 1000, 2000);
  M2.attach(M2_pin, 1000, 2000);
  M3.attach(M3_pin, 1000, 2000);
  M4.attach(M4_pin, 1000, 2000);
  delay(1000);
  Serial.println("Starting ESC Calibration...");
  M1.writeMicroseconds(2000);
  M2.writeMicroseconds(2000);
  M3.writeMicroseconds(2000);
  M4.writeMicroseconds(2000);
  delay(3000);
  M1.writeMicroseconds(1000);
  M2.writeMicroseconds(1000);
  M3.writeMicroseconds(1000);
  M4.writeMicroseconds(1000);
  Serial.println("ESC Calibration Complete!");
  delay(3000);
  M1.writeMicroseconds(1000);
  M2.writeMicroseconds(1000);
  M3.writeMicroseconds(1000);
  M4.writeMicroseconds(1000);
  delay(500);
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);

  server.begin();
  websockets.begin();
  websockets.onEvent(webSocketEvent);
}

// Loop function

void loop() {
  websockets.loop();

  if (!dmpReady || Kill) {
    M1.writeMicroseconds(1000);
    M2.writeMicroseconds(1000);
    M3.writeMicroseconds(1000);
    M4.writeMicroseconds(1000);
    return;
  }
  if (isnan(ypr[0]) || isnan(ypr[1]) || isnan(ypr[2])) {
    // Handle error or skip PID update
    return;
  }

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Serial.print(" ypr\t");
    Serial.print(ypr[0]);
    Serial.print("\t");

    complementaryAngleRoll = ypr[2];
    complementaryAnglePitch = ypr[1];
    complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
    complementaryAnglePitch = (complementaryAnglePitch > 20) ? 20 : ((complementaryAnglePitch < -20) ? -20 : complementaryAnglePitch);
    Serial.print(complementaryAnglePitch);
    Serial.print("\t");
    Serial.print(complementaryAngleRoll);
    Serial.print("\t");
    DesiredAngleRoll = joystick.roll;
    DesiredAnglePitch = joystick.pitch;
    InputThrottle = joystick.throttle;
    DesiredRateYaw = joystick.yaw;


    // Inlined PID equation for Roll
    ErrorAngleRoll = DesiredAngleRoll - complementaryAngleRoll;
    PtermRoll = PAngleRoll * ErrorAngleRoll;
    ItermRoll = PrevItermAngleRoll + (IAngleRoll * (ErrorAngleRoll + PrevErrorAngleRoll) * (t / 2));
    ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
    DtermRoll = DAngleRoll * ((ErrorAngleRoll - PrevErrorAngleRoll) / t);
    PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
    PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);
    DesiredRateRoll = PIDOutputRoll;
    PrevErrorAngleRoll = ErrorAngleRoll;
    PrevItermAngleRoll = ItermRoll;

    ErrorAnglePitch = DesiredAnglePitch - complementaryAnglePitch;
    PtermPitch = PAnglePitch * ErrorAnglePitch;
    ItermPitch = PrevItermAnglePitch + (IAnglePitch * (ErrorAnglePitch + PrevErrorAnglePitch) * (t / 2));
    ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
    DtermPitch = DAnglePitch * ((ErrorAnglePitch - PrevErrorAnglePitch) / t);
    PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
    PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);
    DesiredRatePitch = PIDOutputPitch;
    PrevErrorAnglePitch = ErrorAnglePitch;
    PrevItermAnglePitch = ItermPitch;

    // Compute errors
    ErrorRateRoll = DesiredRateRoll - ypr[2];
    ErrorRatePitch = DesiredRatePitch - ypr[1];
    ErrorRateYaw = DesiredRateYaw - ypr[0];

    // Roll Axis PID
    PtermRoll = PRateRoll * ErrorRateRoll;
    ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
    ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
    DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
    PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
    PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

    // Update output and previous values for Roll
    InputRoll = PIDOutputRoll;
    PrevErrorRateRoll = ErrorRateRoll;
    PrevItermRateRoll = ItermRoll;
    // Pitch Axis PID
    PtermPitch = PRatePitch * ErrorRatePitch;
    ItermPitch = PrevItermRatePitch + (IRatePitch * (ErrorRatePitch + PrevErrorRatePitch) * (t / 2));
    ItermPitch = (ItermPitch > 400) ? 400 : ((ItermPitch < -400) ? -400 : ItermPitch);
    DtermPitch = DRatePitch * ((ErrorRatePitch - PrevErrorRatePitch) / t);
    PIDOutputPitch = PtermPitch + ItermPitch + DtermPitch;
    PIDOutputPitch = (PIDOutputPitch > 400) ? 400 : ((PIDOutputPitch < -400) ? -400 : PIDOutputPitch);

    // Update output and previous values for Pitch
    InputPitch = PIDOutputPitch;
    PrevErrorRatePitch = ErrorRatePitch;
    PrevItermRatePitch = ItermPitch;

    // Yaw Axis PID
    PtermYaw = PRateYaw * ErrorRateYaw;
    ItermYaw = PrevItermRateYaw + (IRateYaw * (ErrorRateYaw + PrevErrorRateYaw) * (t / 2));
    ItermYaw = (ItermYaw > 400) ? 400 : ((ItermYaw < -400) ? -400 : ItermYaw);  // Clamp ItermYaw to [-400, 400]
    DtermYaw = DRateYaw * ((ErrorRateYaw - PrevErrorRateYaw) / t);
    PIDOutputYaw = PtermYaw + ItermYaw + DtermYaw;
    PIDOutputYaw = (PIDOutputYaw > 400) ? 400 : ((PIDOutputYaw < -400) ? -400 : PIDOutputYaw);  // Clamp PIDOutputYaw to [-400, 400]


    // Update output and previous values for Yaw
    InputYaw = PIDOutputYaw;
    PrevErrorRateYaw = ErrorRateYaw;
    PrevItermRateYaw = ItermYaw;


    if (InputThrottle > 1800) {
      InputThrottle = 1800;
    }


    MotorInput1 = (InputThrottle + InputRoll - InputPitch + InputYaw);  // front right - counter clockwise
    MotorInput2 = (InputThrottle - InputRoll - InputPitch - InputYaw);  // rear right - clockwise
    MotorInput3 = (InputThrottle - InputRoll + InputPitch + InputYaw);  // rear left  - counter clockwise
    MotorInput4 = (InputThrottle + InputRoll + InputPitch - InputYaw);  //front left - clockwise
    if (isnan(MotorInput1) || isnan(MotorInput2) || isnan(MotorInput3)||isnan(MotorInput4)) {
      // Handle error or skip PID update
      return;
    }

    if (MotorInput1 > 1800) {
      MotorInput1 = 1600;
    }

    if (MotorInput2 > 1800) {
      MotorInput2 = 1600;
    }

    if (MotorInput3 > 1800) {
      MotorInput3 = 1600;
    }

    if (MotorInput4 > 1800) {
      MotorInput4 = 1600;
    }


    // int 1000 = 1150;
    if (MotorInput1 < 1100) {
      MotorInput1 = 1000;
    }
    if (MotorInput2 < 1100) {
      MotorInput2 = 1000;
    }
    if (MotorInput3 < 1100) {
      MotorInput3 = 1000;
    }
    if (MotorInput4 < 1100) {
      MotorInput4 = 1000;
    }


    // Calculate motor control values directly
    M1.writeMicroseconds(MotorInput1);
    M2.writeMicroseconds(MotorInput2);
    M3.writeMicroseconds(MotorInput3);
    M4.writeMicroseconds(MotorInput4);


    Serial.print("InputThrottle: ");
    Serial.print(InputThrottle);
    Serial.print(" ");
    Serial.print("\t InputPitch: ");
    Serial.print(InputPitch);
    Serial.print(" ");

    Serial.print("\t InputRoll: ");
    Serial.print(InputRoll);
    Serial.print(" ");

    Serial.print("\t InputYaw: ");
    Serial.print(InputYaw);
    Serial.print(" ");


    Serial.print("\t MotorInput1: ");
    Serial.print(MotorInput1);
    Serial.print(" ");

    Serial.print("\t MotorInput2: ");
    Serial.print(MotorInput2);
    Serial.print(" ");

    Serial.print("\t MotorInput3: ");
    Serial.print(MotorInput3);
    Serial.print(" ");

    Serial.print("\t MotorInput4: ");
    Serial.println(MotorInput4);
  }
}
