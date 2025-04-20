#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"
#include <PID_v1.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;
#define INTERRUPT_PIN 25

// PID Variables
double setpointYaw = 0.0, setpointPitch = 0.0, setpointRoll = 0.0;
double inputYaw, inputPitch, inputRoll;
double outputYaw, outputPitch, outputRoll;

// PID tuning parameters
double Kp = 1.5, Ki = 0.05, Kd = 0.1;

// Create PID objects
PID pidYaw(&inputYaw, &outputYaw, &setpointYaw, Kp, Ki, Kd, DIRECT);
PID pidPitch(&inputPitch, &outputPitch, &setpointPitch, Kp, Ki, Kd, DIRECT);
PID pidRoll(&inputRoll, &outputRoll, &setpointRoll, Kp, Ki, Kd, DIRECT);

// Motor PWM values
int motor1, motor2, motor3, motor4;
const int minPWM = 1000, maxPWM = 2000, basePWM = 1500;

// MPU variables
bool dmpReady = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

volatile bool mpuInterrupt = false;
void dmpDataReady() {
  mpuInterrupt = true;
}

void setup() {
  Wire.begin();
  Wire.setClock(100000);
  Serial.begin(115200);
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
    mpu.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  pidYaw.SetMode(AUTOMATIC);
  pidPitch.SetMode(AUTOMATIC);
  pidRoll.SetMode(AUTOMATIC);

  pidYaw.SetOutputLimits(-500, 500);
  pidPitch.SetOutputLimits(-500, 500);
  pidRoll.SetOutputLimits(-500, 500);
}

void loop() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    inputYaw = ypr[0] * 180 / M_PI;
    inputPitch = ypr[1] * 180 / M_PI;
    inputRoll = ypr[2] * 180 / M_PI;

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
    Serial.println(motor4);
  }
}