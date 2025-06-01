#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <ESPAsyncWebServer.h>  //V3.6.0
#include <WebSocketsServer.h>
#include <ArduinoJson.h>  //V7.3.0

// #include <AsyncTCP.h> //V1.1.4
#include <LittleFS.h>
#include <ArduinoJson.h>  //V7.3.0
#include "Wire.h"
#include <WiFi.h>

MPU6050 mpu;
#define INTERRUPT_PIN 23
bool Kill = false;

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
const char* ssid = "Vayu";
const char* password = "123456789";

AsyncWebServer server(80);
WebSocketsServer websockets(81);

// ESC & Motors
Servo M1, M2, M3, M4;
const int M1_pin = 19;
const int M2_pin = 18;
const int M3_pin = 5;
const int M4_pin = 4;
float t = 0.004;

// PID variables
// WEB Variables
const char* PARAM_P_GAIN = "PRatePitch";  //For Pitch & Roll RATE
const char* PARAM_I_GAIN = "IRatePitch";
const char* PARAM_D_GAIN = "DRatePitch";

const char* PARAM_P_A_GAIN = "PAngleRoll";  //For Pitch & Roll ANGLE
const char* PARAM_I_A_GAIN = "IAngleRoll";
const char* PARAM_D_A_GAIN = "DAngleRoll";

const char* PARAM_P_YAW = "PRateYaw";  //For Yaw
const char* PARAM_I_YAW = "IRateYaw";
const char* PARAM_D_YAW = "DRateYaw";

const char* PARAM_TIME_CYCLE = "tc";  //Computation time cycle

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
volatile float PID_Fixed_Roll, Throttle, PID_Fixed_Pitch, PID_Fixed_Yaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

float complementaryAngleRoll = 0.00;
float complementaryAnglePitch = 0.00;

volatile float Motor1_PWM, Motor2_PWM, Motor3_PWM, Motor4_PWM;
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

// ===========================  LittelFS Functions    =============================

String readFile(fs::FS& fs, const char* path) {
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if (!file || file.isDirectory()) {
    Serial.println("- empty file or failed to open file");
    return String();
  }
  // Serial.println("- read from file:");
  String fileContent;
  while (file.available()) {
    fileContent += String((char)file.read());
  }
  file.close();
  // Serial.println(fileContent);
  return fileContent;
}

void writeFile(fs::FS& fs, const char* path, const char* message) {
  Serial.printf("Writing file: %s\r\n", path);
  File file = fs.open(path, "w");
  if (!file) {
    Serial.println("- failed to open file for writing");
    return;
  }
  if (file.print(message)) {
    // Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}

// ===========================  Web Functions    =============================

void sendtxt(String txt) {
  Serial.println(txt);
  websockets.sendTXT(0, txt);
}
void webSocketEvent(uint8_t clientId, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_CONNECTED) {
    IPAddress ip = websockets.remoteIP(clientId);
    Serial.printf("[%u] Connected from %d.%d.%d.%d\n", clientId, ip[0], ip[1], ip[2], ip[3]);
  } else if (type == WStype_TEXT) {
    String receivedPayload((char*)payload, length);
    // Serial.printf("Received from client [%u]: %s\n", clientId, receivedPayload.c_str());
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
      // joystick.throttle = constrain(joystick.throttle, 1000, 2000);
      joystick.pitch = message["pitch"];
      joystick.roll = message["roll"];
      joystick.yaw = message["yaw"];
    }
  }
}
String processor(const String& var) {
  if (var == "pGain") {
    return readFile(LittleFS, "/pGain.txt");
  } else if (var == "iGain") {
    return readFile(LittleFS, "/iGain.txt");
  } else if (var == "dGain") {
    return readFile(LittleFS, "/dGain.txt");
  } else if (var == "pAGain") {
    return readFile(LittleFS, "/pAGain.txt");
  } else if (var == "iAGain") {
    return readFile(LittleFS, "/iAGain.txt");
  } else if (var == "dAGain") {
    return readFile(LittleFS, "/dAGain.txt");
  } else if (var == "pYaw") {
    return readFile(LittleFS, "/pYaw.txt");
  } else if (var == "dYaw") {
    return readFile(LittleFS, "/dYaw.txt");
  } else if (var == "iYaw") {
    return readFile(LittleFS, "/iYaw.txt");
  } else if (var == "tc") {
    return readFile(LittleFS, "/tc.txt");
  }
}

// ===========================  Calibration and MPU Functions    =============================

void dmpDataReady() {
  mpuInterrupt = true;
}

void calibrate_esc() {
  M1.writeMicroseconds(2000);
  M2.writeMicroseconds(2000);
  M3.writeMicroseconds(2000);
  M4.writeMicroseconds(2000);
  Serial.println("Sending MAX signal (2000)... wait for beeps...");
  delay(3000);  // Wait for beeps

  M1.writeMicroseconds(1000);
  M2.writeMicroseconds(1000);
  M3.writeMicroseconds(1000);
  M4.writeMicroseconds(1000);
  Serial.println("Sending MIN signal (1000)... wait for confirmation tones...");
  delay(3000);  // Wait for arming tones

  Serial.println("Calibration done. ESC is now armed.");
  M1.writeMicroseconds(1000);
  M2.writeMicroseconds(1000);
  M3.writeMicroseconds(1000);
  M4.writeMicroseconds(1000);
  delay(500);
}

void mpu_setup() {
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
}

// ===========================  Setup  =============================

void setup() {
  Serial.begin(115200);
  // ================================= File System Intialization =========================

  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }

  // ================================= Wifi Intialization =========================

  if (!WiFi.softAP(ssid, password)) {
    Serial.println("WiFi Failed!");
    return;
  }
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  // ================================= WEB Routes =========================

  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(LittleFS, "/index.html", "text/html", false);
  });
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest* request) {
    String inputMessage;
    // GET P Gain value on <ESP_IP>/get?pGain=<inputMessage>
    if (request->hasParam(PARAM_P_GAIN)) {
      inputMessage = request->getParam(PARAM_P_GAIN)->value();
      writeFile(LittleFS, "/pGain.txt", inputMessage.c_str());
      PRateRoll = readFile(LittleFS, "/pGain.txt").toFloat();
      PRatePitch = PRateRoll;
    } else if (request->hasParam(PARAM_I_GAIN)) {
      inputMessage = request->getParam(PARAM_I_GAIN)->value();
      writeFile(LittleFS, "/iGain.txt", inputMessage.c_str());
      IRateRoll = readFile(LittleFS, "/iGain.txt").toFloat();
      IRatePitch = IRateRoll;
    } else if (request->hasParam(PARAM_D_GAIN)) {
      inputMessage = request->getParam(PARAM_D_GAIN)->value();
      writeFile(LittleFS, "/dGain.txt", inputMessage.c_str());
      DRateRoll = readFile(LittleFS, "/dGain.txt").toFloat();
      DRatePitch = DRateRoll;
    } else if (request->hasParam(PARAM_P_A_GAIN)) {
      inputMessage = request->getParam(PARAM_P_A_GAIN)->value();
      writeFile(LittleFS, "/pAGain.txt", inputMessage.c_str());
      PAngleRoll = readFile(LittleFS, "/pAGain.txt").toFloat();
      PAnglePitch = PAngleRoll;
    } else if (request->hasParam(PARAM_I_A_GAIN)) {
      inputMessage = request->getParam(PARAM_I_A_GAIN)->value();
      writeFile(LittleFS, "/iAGain.txt", inputMessage.c_str());
      IAngleRoll = readFile(LittleFS, "/iAGain.txt").toFloat();
      IAnglePitch = IAngleRoll;
    } else if (request->hasParam(PARAM_D_A_GAIN)) {
      inputMessage = request->getParam(PARAM_D_A_GAIN)->value();
      writeFile(LittleFS, "/dAGain.txt", inputMessage.c_str());
      DAngleRoll = readFile(LittleFS, "/dAGain.txt").toFloat();
      DAnglePitch = DAngleRoll;
    } else if (request->hasParam(PARAM_P_YAW)) {
      inputMessage = request->getParam(PARAM_P_YAW)->value();
      writeFile(LittleFS, "/pYaw.txt", inputMessage.c_str());
      PRateYaw = readFile(LittleFS, "/pYaw.txt").toFloat();
    } else if (request->hasParam(PARAM_I_YAW)) {
      inputMessage = request->getParam(PARAM_I_YAW)->value();
      writeFile(LittleFS, "/iYaw.txt", inputMessage.c_str());
      IRateYaw = readFile(LittleFS, "/iYaw.txt").toFloat();
    } else if (request->hasParam(PARAM_D_YAW)) {
      inputMessage = request->getParam(PARAM_D_YAW)->value();
      writeFile(LittleFS, "/dYaw.txt", inputMessage.c_str());
      DRateYaw = readFile(LittleFS, "/dYaw.txt").toFloat();
    } else if (request->hasParam(PARAM_TIME_CYCLE)) {
      inputMessage = request->getParam(PARAM_TIME_CYCLE)->value();
      writeFile(LittleFS, "/tc.txt", inputMessage.c_str());
      t = inputMessage.toFloat();
    } else {
      inputMessage = "No message sent";
    }
    Serial.print("inputMessage ");
    Serial.print(inputMessage);
    Serial.println();
    if (inputMessage == "") {
      return;
    }
    StaticJsonDocument<2048> jsonDoc;
    JsonObject status = jsonDoc.createNestedObject();
    status["action"] = "GAIN SET";
    status["PRateRoll"] = PRateRoll;
    status["IRateRoll"] = IRateRoll;
    status["DRateRoll"] = DRateRoll;
    status["PRatePitch"] = PRatePitch;
    status["IRatePitch"] = IRatePitch;
    status["DRatePitch"] = DRatePitch;
    status["PAngleRoll"] = PAngleRoll;
    status["IAngleRoll"] = IAngleRoll;
    status["DAngleRoll"] = DAngleRoll;
    status["PAnglePitch"] = PAnglePitch;
    status["IAnglePitch"] = IAnglePitch;
    status["DAnglePitch"] = DAnglePitch;
    status["PRateYaw"] = PRateYaw;
    status["IRateYaw"] = IRateYaw;
    status["DRateYaw"] = DRateYaw;
    String jsonString;
    serializeJson(jsonDoc, jsonString);
    sendtxt(jsonString);
    request->send(200, "text/text", inputMessage);
  });
  server.serveStatic("/", LittleFS, "/");
  server.begin();
  websockets.begin();
  websockets.onEvent(webSocketEvent);
  Serial.println("Web Server Started");

  // ================================= Sensor and Esc Intialization =========================

  mpu_setup();
  M1.attach(M1_pin, 1000, 2000);
  M2.attach(M2_pin, 1000, 2000);
  M3.attach(M3_pin, 1000, 2000);
  M4.attach(M4_pin, 1000, 2000);
  delay(100);
  calibrate_esc();
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
    // Serial.print(" ypr\t");
    // Serial.print((ypr[0] * 180) / 3.14);
    // Serial.print("\t");

    complementaryAngleRoll = (ypr[2] * 180) / 3.14;
    complementaryAnglePitch = (ypr[1] * 180) / 3.14;
    // complementaryAngleRoll = (complementaryAngleRoll > 20) ? 20 : ((complementaryAngleRoll < -20) ? -20 : complementaryAngleRoll);
    // complementaryAngleRoll = (ypr[2] > 0.05 || (ypr[2] < -0.05)) ? ypr[2]:complementaryAngleRoll;
    // complementaryAnglePitch = (ypr[1] > 0.05 || (ypr[1] < -0.05)) ? ypr[1]:complementaryAnglePitch;
    // Serial.print(complementaryAnglePitch);
    // Serial.print("\t");
    // Serial.print(complementaryAngleRoll);
    // Serial.print("\t");
    DesiredAngleRoll = joystick.roll;
    DesiredAnglePitch = joystick.pitch;
    Throttle = joystick.throttle;
    DesiredRateYaw = joystick.yaw;

    // Outer Loop PID to set desiered setpoint
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

    // Inner Loop PID to set desiered setpoint
    PtermRoll = PRateRoll * ErrorRateRoll;
    ItermRoll = PrevItermRateRoll + (IRateRoll * (ErrorRateRoll + PrevErrorRateRoll) * (t / 2));
    ItermRoll = (ItermRoll > 400) ? 400 : ((ItermRoll < -400) ? -400 : ItermRoll);
    DtermRoll = DRateRoll * ((ErrorRateRoll - PrevErrorRateRoll) / t);
    PIDOutputRoll = PtermRoll + ItermRoll + DtermRoll;
    PIDOutputRoll = (PIDOutputRoll > 400) ? 400 : ((PIDOutputRoll < -400) ? -400 : PIDOutputRoll);

    // Update output and previous values for Roll
    PID_Fixed_Roll = PIDOutputRoll;
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
    PID_Fixed_Pitch = PIDOutputPitch;
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
    PID_Fixed_Yaw = PIDOutputYaw;
    PrevErrorRateYaw = ErrorRateYaw;
    PrevItermRateYaw = ItermYaw;

    if (Throttle > 1800) {
      Throttle = 1800;
    }

    Motor1_PWM = (Throttle + PID_Fixed_Roll - PID_Fixed_Pitch + PID_Fixed_Yaw);  // front right - counter clockwise
    Motor2_PWM = (Throttle - PID_Fixed_Roll - PID_Fixed_Pitch - PID_Fixed_Yaw);  // rear right - clockwise
    Motor3_PWM = (Throttle - PID_Fixed_Roll + PID_Fixed_Pitch + PID_Fixed_Yaw);  // rear left  - counter clockwise
    Motor4_PWM = (Throttle + PID_Fixed_Roll + PID_Fixed_Pitch - PID_Fixed_Yaw);  // front left - clockwise
    if (isnan(Motor1_PWM) || isnan(Motor2_PWM) || isnan(Motor3_PWM) || isnan(Motor4_PWM)) {
      return;
    }

    if (Motor1_PWM > 1800) {
      Motor1_PWM = 1600;
    }

    if (Motor2_PWM > 1800) {
      Motor2_PWM = 1600;
    }

    if (Motor3_PWM > 1800) {
      Motor3_PWM = 1600;
    }

    if (Motor4_PWM > 1800) {
      Motor4_PWM = 1600;
    }

    if (Motor1_PWM < 1100) {
      Motor1_PWM = 1000;
    }
    if (Motor2_PWM < 1100) {
      Motor2_PWM = 1000;
    }
    if (Motor3_PWM < 1100) {
      Motor3_PWM = 1000;
    }
    if (Motor4_PWM < 1100) {
      Motor4_PWM = 1000;
    }

    // Calculate motor control values directly
    M1.writeMicroseconds(Motor1_PWM);
    M2.writeMicroseconds(Motor2_PWM);
    M3.writeMicroseconds(Motor3_PWM);
    M4.writeMicroseconds(Motor4_PWM);

    // Serial.print("Throttle: ");
    // Serial.print(Throttle);
    // Serial.print(" ");
    // Serial.print("\t PID_Fixed_Pitch: ");
    // Serial.print(PID_Fixed_Pitch);
    // Serial.print(" ");

    // Serial.print("\t PID_Fixed_Roll: ");
    // Serial.print(PID_Fixed_Roll);
    // Serial.print(" ");

    // Serial.print("\t PID_Fixed_Yaw: ");
    // Serial.print(PID_Fixed_Yaw);
    // Serial.print(" ");

    // Serial.print("\t Motor1_PWM: ");
    // Serial.print(Motor1_PWM);
    // Serial.print(" ");

    // Serial.print("\t Motor2_PWM: ");
    // Serial.print(Motor2_PWM);
    // Serial.print(" ");

    // Serial.print("\t Motor3_PWM: ");
    // Serial.print(Motor3_PWM);
    // Serial.print(" ");

    // Serial.print("\t Motor4_PWM: ");
    // Serial.println(Motor4_PWM);
  }
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
//   PID_Fixed_Yaw = ypr[0] * 180 / M_PI;
//   PID_Fixed_Pitch = ypr[1] * 180 / M_PI;
//   PID_Fixed_Roll = ypr[2] * 180 / M_PI;

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