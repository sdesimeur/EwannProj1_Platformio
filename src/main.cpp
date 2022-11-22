//#define ICACHE_RAM_ATTR

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
//#if I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
#include "Wire.h"
#endif
#include <DNSServer.h>
//#define DEBUG_ESP_PORT Serial
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "common.h"
#include "index.h"

MPU6050 mpu;

#define OUTPUT_READABLE_WORLDACCEL

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector

bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

const int chipSelect = 4;

//int startButtonState = 0;
//const int startButtontPin = 8;  // # Faltou setar como output
int isRecording = 1;            // if odd, then it is not recording; if even it is.
//bool isStartButtonHigh = false; // to prevent repeated HIGH detecttion for the button pin //#
int accelX;
int accelY;
int accelZ;
char data[64]; // MPU data to be written to SD card //#

char dataTime[64];              // A separator for millis() data  //#
unsigned long ellapsedTime = 0; // Contais millis() data //#

bool fifoOverflow = false; // Red light.
bool canRecord = false;    // Blue light. Timer will confirm if it is ok to start recording
                        // isRecording is green light
bool magnetoFieldInit = false;
double magnetoField = 1000000;
double magnetoFieldMean = 0;
double magnetoFieldMax = -1000000;
double magnetoFieldMin = 1000000;

DNSServer dnsServer;
ESP8266WebServer server(80);

//void ICACHE_RAM_ATTR handleInterrupt();

void handleRoot() {
  server.send(200, "text/html", INDEX_HTML);
}

void IRAM_ATTR dmpDataReady()
{
  mpuInterrupt = true;
}
double get_accel_abs () {
    double ret = 0;
    ret += accelX * accelX;
    ret += accelY * accelY;
    ret += accelZ * accelZ;
    ret = sqrt(ret);
    return ret;
}

void handleConfigMagneto()
{
  if (server.method() != HTTP_GET) {
    server.send(405, "text/plain", "Method Not Allowed");
  } else if (server.argName(0).compareTo("config")) {
    magnetoFieldInit = server.arg(0).compareTo("0")?false:true;
    server.send(200, "text/plain", "1");
  } else {
    server.send(405, "text/plain", "Arg error");
  }
}
void handleMagnetField () {
  if (server.method() != HTTP_GET) {
    server.send(405, "text/plain", "Method Not Allowed");
  } else if (server.argName(0).compareTo("w")) {
    String ret = "";
    String arg = server.arg(0);
    switch (arg.charAt(0)) {
      case 'M':
      ret += magnetoFieldMax;
      break;
      case 'm':
      ret += magnetoFieldMin;
      break;
      case '2':
      ret += magnetoFieldMean;
      break;
      case 'n':
      default:
      ret += magnetoField;
      break;
    }
    server.send(200, "text/plain", ret);
  } else {
    server.send(405, "text/plain", "Arg error");
  }
}
void handleAccel () {
  if (server.method() != HTTP_GET) {
    server.send(405, "text/plain", "Method Not Allowed");
  } else {
    String ret = "";
    ret += get_accel_abs();
    server.send(200, "text/plain", ret);
  }
}
void handleNotFound() {
  String message = "File Not Found\n\n";
  server.send(404, "text/plain", message);
}
void setup()
{
  Serial.begin(115200);
  pinMode(15, INPUT_PULLDOWN_16);
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  delay(3000);
  digitalWrite(5, HIGH);
  delay(1000);
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(12, 14);
  Wire.setClock(400000L);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  WiFi.softAP("Ewann");
  WiFi.softAPConfig(IPAddress(192, 192, 192, 1), IPAddress(0, 0, 0, 0), IPAddress(255, 255, 255, 0));
  // wifi_softap_dhcps_start();
  WiFi.begin(EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
  WiFi.config(IPAddress(192, 168, 4, 245), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  dnsServer.start(53, "*", WiFi.softAPIP());

  server.on("/", handleRoot);
  //server.on("/rpc/getSpeed", handleGetSpeed);
  server.on("/rpc/getAccel", handleAccel);
  //server.on("/rpc/getRawAccel", handleRawAccel);
  server.on("/rpc/getMagnetField", handleMagnetField);
  //server.on("/rpc/initAccelSpeed", handleInitAccelSpeed);
  server.on("/rpc/configMagneto", handleConfigMagneto);
  server.onNotFound(handleNotFound);
  server.begin();

  mpu.initialize();
  //mpu.testConnection();

  devStatus = mpu.dmpInitialize();
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);

  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    attachInterrupt(digitalPinToInterrupt(15), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.print(F("DMP Initialization OK"));
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
}
void readMagnetoField () {
  double val = 1000000;
  double tmp = 0;
  double valMin = 1000000;
  double valMax = 1000000;
  double valMean = 1000000;
  while (true) {
    tmp = (double)analogRead(0);
    tmp *= 3;
    tmp *= 3.3;
    tmp /= 2048;
    tmp /= 18;
    if (!magnetoFieldInit) {
        valMin = 1000000;
        valMax = -1000000;
    } else {
        if (valMax < tmp) valMax = tmp;
        if (valMin > tmp) valMin = tmp;
        valMean = (valMax + valMin) / 2;
    }
    tmp -= valMean;
    if (val == 1000000) {
        val = tmp;
    } else {
        val *= 3;
        val += tmp;
        val /= 4;
    }
    magnetoField = val;
    magnetoFieldMax = valMax;
    magnetoFieldMin = valMin;
    magnetoFieldMean = valMean;
    delay(100);
  }
}

void loop()
{
  dnsServer.processNextRequest();
  server.handleClient();
  //readMagnetoField();
#if 0
    Serial.println("loop");
    Serial.print("f canRecord: ");
    Serial.print(canRecord);
    Serial.print("startButtonState: ");
    Serial.print(startButtonState);
    Serial.print("isStartButtonHigh: ");
    Serial.print(isStartButtonHigh);
    Serial.print("isRecording: ");
    Serial.println(isRecording);
    Serial.println("\n");
#endif

  ellapsedTime = millis();

  if (ellapsedTime > 10000)
  {                   // Wait until 60 seconds for MPU data stabilize //# Só espera 5 segundos
    canRecord = true; // Now data can be recorded
  }

  // MPU CODE STARTS HERE
  if (!dmpReady)
    return;

  while (!mpuInterrupt && fifoCount < packetSize)
  {
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();

    fifoOverflow = true;
    Serial.println("OVERFLOW");
  }
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize)
      fifoCount = mpu.getFIFOCount();

    mpu.getFIFOBytes(fifoBuffer, packetSize);

    fifoCount -= packetSize;

#ifdef OUTPUT_READABLE_WORLDACCEL
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
#endif
    if (canRecord)
    { // do not record until at least 60 seconds are passed
      //startButtonState = digitalRead(startButtontPin);

      accelX = aaWorld.x;
      accelY = aaWorld.y;
      accelZ = aaWorld.z;

      //sprintf(data, "%d\t%d\t%d", accelX, accelY, accelZ);
      //Serial.print(data);
    }
  }

#if 0  
    Serial.print("f canRecord: ");
    Serial.print(canRecord);
    Serial.print("startButtonState: ");
    Serial.print(startButtonState);
    Serial.print("isStartButtonHigh: ");
    Serial.print(isStartButtonHigh);
    Serial.print("isRecording: ");
    Serial.println(isRecording);
    Serial.println("\n");
#endif
}
