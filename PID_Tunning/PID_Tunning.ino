#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <LittleFS.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

bool Kill = false;
// MPU variables
MPU6050 mpu;
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];
#define INTERRUPT_PIN 23
Quaternion q;
VectorFloat gravity;
float ypr[3];

// Interrupt detection
volatile bool mpuInterrupt = false;

// REPLACE WITH YOUR NETWORK CREDENTIALS
//COnnect your PC/mobile to this wifi and open the IP address from the serial monitor in your browser.
const char* ssid = "MATAJI";
const char* password = "anshibai@1";
volatile int webThrottle = 1000;
unsigned long lastWebCheck = 0;
const unsigned long webTimeout = 5000;
float PRateRoll = 0.625;
float IRateRoll = 2.1;
float DRateRoll = 0.008;
float PRatePitch = PRateRoll;
float IRatePitch = IRateRoll;
float DRatePitch = DRateRoll;

float PAngleRoll = 2;
float IAngleRoll = 0;
float DAngleRoll = 0.007;
float PAnglePitch = PAngleRoll;
float IAnglePitch = IAngleRoll;
float DAnglePitch = DAngleRoll;

float PRateYaw = 4;
float IRateYaw = 3;
float DRateYaw = 0;

int ESCfreq = 500;

float t = 0.004;  //time cycle

Servo mot1;
Servo mot2;
Servo mot3;
Servo mot4;
const int mot1_pin = 19;
const int mot2_pin = 18;
const int mot3_pin = 5;  //14 for perf board
const int mot4_pin = 4;

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

int ThrottleIdle = 1000;

volatile float DesiredRateRoll, DesiredRatePitch, DesiredRateYaw;
volatile float ErrorRateRoll, ErrorRatePitch, ErrorRateYaw;
volatile float InputRoll, InputThrottle, InputPitch, InputYaw;
volatile float PrevErrorRateRoll, PrevErrorRatePitch, PrevErrorRateYaw;
volatile float PrevItermRateRoll, PrevItermRatePitch, PrevItermRateYaw;
volatile float PIDReturn[] = { 0, 0, 0 };

float complementaryAngleRoll = 0.0f;
float complementaryAnglePitch = 0.0f;

volatile float DesiredAngleRoll, DesiredAnglePitch;
volatile float ErrorAngleRoll, ErrorAnglePitch;
volatile float PrevErrorAngleRoll, PrevErrorAnglePitch;
volatile float PrevItermAngleRoll, PrevItermAnglePitch;

volatile int MotorInput1, MotorInput2, MotorInput3, MotorInput4;




//WIFI tuning global code
AsyncWebServer server(80);


const char* PARAM_THROTTEL = "throttel";  //For Pitch & Roll RATE
const char* PARAM_P_GAIN = "pGain";       //For Pitch & Roll RATE
const char* PARAM_I_GAIN = "iGain";
const char* PARAM_D_GAIN = "dGain";

const char* PARAM_P_A_GAIN = "pAGain";  //For Pitch & Roll ANGLE
const char* PARAM_I_A_GAIN = "iAGain";
const char* PARAM_D_A_GAIN = "dAGain";

const char* PARAM_P_YAW = "pYaw";  //For Yaw
const char* PARAM_I_YAW = "iYaw";
const char* PARAM_D_YAW = "dYaw";

const char* PARAM_TIME_CYCLE = "tc";  //Computation time cycle


void notFound(AsyncWebServerRequest* request) {
  request->send(404, "text/plain", "Not found");
}
void dmpDataReady() {
  mpuInterrupt = true;
}
String readFile(fs::FS& fs, const char* path) {
  Serial.printf("Reading file: %s\r\n", path);
  File file = fs.open(path, "r");
  if (!file || file.isDirectory()) {
    Serial.println("- empty file or failed to open file");
    return String();
  }
  Serial.println("- read from file:");
  String fileContent;
  while (file.available()) {
    fileContent += String((char)file.read());
  }
  file.close();
  Serial.println(fileContent);
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
    Serial.println("- file written");
  } else {
    Serial.println("- write failed");
  }
  file.close();
}
// Replaces placeholder with stored values
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
void setup(void) {

  Serial.begin(115200);
  pinMode(2, OUTPUT);

  if (!LittleFS.begin(true)) {
    Serial.println("An Error has occurred while mounting LittleFS");
    return;
  }
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("WiFi Failed!");
    return;
  }
  Serial.println();
  Serial.print("IP Address: ");

  Serial.println(WiFi.localIP());
  // Send web page with input fields to client
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    request->send(LittleFS, "/index1.html", "text/html", false, processor);
  });
  server.serveStatic("/", LittleFS, "/");
  // Send a GET request to <ESP_IP>/get?inputString=<inputMessage>
  server.on("/get", HTTP_GET, [](AsyncWebServerRequest* request) {
    String inputMessage;
    // GET P Gain value on <ESP_IP>/get?pGain=<inputMessage>
    if (request->hasParam("KILL")) {
      Kill = true;
    } else if (request->hasParam(PARAM_THROTTEL)) {
      inputMessage = request->getParam(PARAM_THROTTEL)->value();
      if (inputMessage.c_str() != "") {
        webThrottle = inputMessage.toInt();
        webThrottle = constrain(webThrottle, 1000, 1800);
      }
    } else if (request->hasParam(PARAM_P_GAIN)) {
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
    // Serial.print("inputMessage ");
    // Serial.print(inputMessage);
    // Serial.print(" ");
    request->send(200, "text/text", inputMessage);
  });

  server.onNotFound(notFound);
  server.begin();
  //WIFI server setup END

  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);
  mot3.attach(mot3_pin, 1000, 2000);
  mot4.attach(mot4_pin, 1000, 2000);
  delay(100);

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
  mot4.writeMicroseconds(1000);
  delay(500);
  digitalWrite(2, HIGH);
  delay(500);
  digitalWrite(2, LOW);
  delay(500);

  // mpu setup
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

void loop(void) {
  if (!dmpReady)
    return;
  if (Kill)
    return;

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
    DesiredAngleRoll = 0.1 * 0;
    DesiredAnglePitch = 0.1 * 0;
    InputThrottle = webThrottle;
    DesiredRateYaw = 0.15 * 0;


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

    if (MotorInput1 > 2000) {
      MotorInput1 = 1800;
    }

    if (MotorInput2 > 2000) {
      MotorInput2 = 1800;
    }

    if (MotorInput3 > 2000) {
      MotorInput3 = 1800;
    }

    if (MotorInput4 > 2000) {
      MotorInput4 = 1800;
    }


    // int ThrottleIdle = 1150;
    if (MotorInput1 < 1100) {
      MotorInput1 = ThrottleIdle;
    }
    if (MotorInput2 < 1100) {
      MotorInput2 = ThrottleIdle;
    }
    if (MotorInput3 < 1100) {
      MotorInput3 = ThrottleIdle;
    }
    if (MotorInput4 < 1100) {
      MotorInput4 = ThrottleIdle;
    }


    // Calculate motor control values directly
    mot1.writeMicroseconds(MotorInput1);
    mot2.writeMicroseconds(MotorInput2);
    mot3.writeMicroseconds(MotorInput3);
    mot4.writeMicroseconds(MotorInput4);

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