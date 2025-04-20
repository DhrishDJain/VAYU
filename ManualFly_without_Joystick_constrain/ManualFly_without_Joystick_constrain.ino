#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>  // Captive portal
#include <LittleFS.h>   // File server
#include <ESP32Servo.h> // Servo library for ESC control

// Access Point SSID & Password
const char *ssid = "QuadcopterControl"; 
const char *password = NULL;  

// WebServer instance
WebServer server(80);
DNSServer dnsServer; 

// ESC & Motor Pins
int ESCfreq = 500;
Servo mot1, mot2, mot3, mot4;
const int mot1_pin = 13, mot2_pin = 12, mot3_pin = 14, mot4_pin = 27;

struct JoystickData {
  int roll, pitch, throttle, yaw;
};
JoystickData joystick;

// **🔧 ESC Calibration (Corrected)**
void escCalibration() {
  Serial.println("Starting ESC Calibration...");
  
  mot1.writeMicroseconds(2000);
  mot2.writeMicroseconds(2000);
  mot3.writeMicroseconds(2000);
  mot4.writeMicroseconds(2000);
  Serial.println("Throttle MAX → Turn on ESCs now.");
  delay(7000);  

  mot1.writeMicroseconds(1000);
  mot2.writeMicroseconds(1000);
  mot3.writeMicroseconds(1000);
  mot4.writeMicroseconds(1000);
  Serial.println("Throttle MIN → Calibration Done.");
  delay(7000);
}

int mapSpeedToPWM(int speedPercent) {
  return map(speedPercent, 0, 1000, 1000, 2000);
}

// **📡 Handle Joystick Data**
void handleJoystick() {
  if (server.args() == 4) {
    joystick.roll = server.arg("roll").toInt();
    joystick.pitch = server.arg("pitch").toInt();
    joystick.throttle = server.arg("throttle").toInt();
    joystick.yaw = server.arg("yaw").toInt();
    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "Invalid Request");
  }
}

// **🌐 Serve Files from LittleFS**
void handleFileServer() {
  String path = server.uri();
  if (path == "/") path = "/index.html";

  String contentType = "text/plain";
  if (path.endsWith(".html")) contentType = "text/html";
  else if (path.endsWith(".css")) contentType = "text/css";
  else if (path.endsWith(".js")) contentType = "application/javascript";

  if (LittleFS.exists(path)) {
    File file = LittleFS.open(path, "r");
    server.streamFile(file, contentType);
    file.close();
  } else {
    server.send(404, "text/plain", "NOT FOUND");
  }
}

void setup() {
  Serial.begin(115200);
  escCalibration();

  if (!LittleFS.begin()) {
    Serial.println("Error mounting LittleFS");
    while (true);
  }
  Serial.println("LittleFS mounted successfully");

  WiFi.softAP(ssid, password);
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  dnsServer.start(53, "*", WiFi.softAPIP());

  // WebServer Routes
  server.on("/joystick", HTTP_GET, handleJoystick);
  server.on("/", handleFileServer);
  server.onNotFound(handleFileServer);
  server.begin();
  Serial.println("Web server started");

  // **ESC Setup**
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  delay(1000);
  mot1.attach(mot1_pin, 1000, 2000);
  mot2.attach(mot2_pin, 1000, 2000);
  mot3.attach(mot3_pin, 1000, 2000);
  mot4.attach(mot4_pin, 1000, 2000);
  delay(100);

  // Set Initial Throttle to **1000 µs (Motor OFF)**
  int pwmValue = mapSpeedToPWM(0);
  mot1.writeMicroseconds(pwmValue);
  mot2.writeMicroseconds(pwmValue);
  mot3.writeMicroseconds(pwmValue);
  mot4.writeMicroseconds(pwmValue);
}

void loop() {
  server.handleClient();
  dnsServer.processNextRequest();

  int speed = constrain(joystick.throttle, 0, 1000);
  int pwmValue = mapSpeedToPWM(speed);

  mot1.writeMicroseconds(pwmValue);
  mot2.writeMicroseconds(pwmValue);
  mot3.writeMicroseconds(pwmValue);
  mot4.writeMicroseconds(pwmValue);

  Serial.print("Throttle: "); Serial.print(speed);
  Serial.print(" | PWM: "); Serial.println(pwmValue);
  delay(10);
}
