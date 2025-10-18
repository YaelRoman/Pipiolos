#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <MPU9250_asukiaaa.h>

// ----------------- LoRa -----------------
#define ss   5
#define rst  14
#define dio0 2

// ---------------- BME280 ----------------
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// ----------------- GPS -----------------
#define RXD2 16
#define TXD2 17
#define GPS_BAUD 9600
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// ----------------- MPU9250 -----------------
#ifdef _ESP32_HAL_I2C_H_
#define SDA_PIN 21
#define SCL_PIN 22
#endif
MPU9250_asukiaaa mpu;

// -------- PACKET STRUCT ------------
struct SensorPacket {
  float temperature;
  float pressure;
  float altitude;
  float humidity;

  float latitude;
  float longitude;
  float speed;
  float gpsAltitude;
  float hdop;
  uint8_t satellites;

  float accelX, accelY, accelZ, accelSqrt;
  float gyroX, gyroY, gyroZ;
  float magX, magY, magZ;
  float magDir;
};

SensorPacket packet;

int counter = 0;
unsigned long delayTime = 2000;

// ----------- FUNCTIONS -----------

// Fill BME280 data
void readBME280() {
  packet.temperature = bme.readTemperature();
  packet.pressure    = bme.readPressure() / 100.0F;
  packet.altitude    = bme.readAltitude(SEALEVELPRESSURE_HPA);
  packet.humidity    = bme.readHumidity();
}

// Fill GPS data
void readGPS() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    packet.latitude  = gps.location.lat();
    packet.longitude = gps.location.lng();
  } else {
    packet.latitude = 0;
    packet.longitude = 0;
  }

  packet.speed      = gps.speed.kmph();
  packet.gpsAltitude= gps.altitude.meters();
  packet.hdop       = gps.hdop.isValid() ? gps.hdop.hdop() : 0;
  packet.satellites = gps.satellites.isValid() ? gps.satellites.value() : 0;
}

// Fill MPU data
void readMPU9250() {
  mpu.accelUpdate();
  mpu.gyroUpdate();
  mpu.magUpdate();

  packet.accelX    = mpu.accelX();
  packet.accelY    = mpu.accelY();
  packet.accelZ    = mpu.accelZ();
  packet.accelSqrt = mpu.accelSqrt();

  packet.gyroX = mpu.gyroX();
  packet.gyroY = mpu.gyroY();
  packet.gyroZ = mpu.gyroZ();

  packet.magX  = mpu.magX();
  packet.magY  = mpu.magY();
  packet.magZ  = mpu.magZ();
  packet.magDir= mpu.magHorizDirection();
}

// Print packet content (debugging)
void printPacket(const SensorPacket& p) {
  Serial.println("==== PACKET ====");
  Serial.print("Temp: "); Serial.println(p.temperature);
  Serial.print("Pres: "); Serial.println(p.pressure);
  Serial.print("Alt:  "); Serial.println(p.altitude);
  Serial.print("Hum:  "); Serial.println(p.humidity);

  Serial.print("Lat:  "); Serial.println(p.latitude, 6);
  Serial.print("Lon:  "); Serial.println(p.longitude, 6);
  Serial.print("Speed:"); Serial.println(p.speed);
  Serial.print("GPS Alt: "); Serial.println(p.gpsAltitude);
  Serial.print("HDOP: "); Serial.println(p.hdop);
  Serial.print("Sats: "); Serial.println(p.satellites);

  Serial.print("Accel XYZ: "); Serial.print(p.accelX); Serial.print(", ");
  Serial.print(p.accelY); Serial.print(", "); Serial.println(p.accelZ);
  Serial.print("Accel Sqrt: "); Serial.println(p.accelSqrt);

  Serial.print("Gyro XYZ: "); Serial.print(p.gyroX); Serial.print(", ");
  Serial.print(p.gyroY); Serial.print(", "); Serial.println(p.gyroZ);

  Serial.print("Mag XYZ: "); Serial.print(p.magX); Serial.print(", ");
  Serial.print(p.magY); Serial.print(", "); Serial.println(p.magZ);
  Serial.print("Mag Dir: "); Serial.println(p.magDir);
  Serial.println("================");
}

// ----------------- SETUP -----------------
void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.println("LoRa + BME280 + GPS + MPU9250 Sender (Binary Struct)");

  // BME280
  if(!bme.begin(0x76)) {
    Serial.println("BME280 error"); while(1);
  } else {
    Serial.println("BME280 ✔");
  }

  // GPS
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  // Validation: Wait some seconds to get the first fix
  unsigned long startTime = millis();
  bool gpsReady = false;
  while (millis() - startTime < 5000) { 
    while (gpsSerial.available() > 0) {
      gps.encode(gpsSerial.read());
    }
    if (gps.location.isValid()) {
      gpsReady = true;
      break;
    }
  }
  if (gpsReady) {
    Serial.println("GPS Initialized ✔");
  } else {
    Serial.println("GPS ERROR: No fix detected!");
  }


  // MPU9250
  Wire.begin(SDA_PIN, SCL_PIN);
  mpu.setWire(&Wire);
  uint8_t sensorId;
  if (mpu.readId(&sensorId) == 0) {
    Serial.println("MPU9250 ✔ ID = " + String(sensorId));
  } else {
    Serial.println("MPU9250 ERROR");
  }
  mpu.beginAccel();
  mpu.beginGyro();
  mpu.beginMag();

  // LoRa
  LoRa.setPins(ss, rst, dio0);
  while(!LoRa.begin(433E6)) {
    Serial.println("LoRa init...");
    delay(500);
  }
  LoRa.setSpreadingFactor(7);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa ✔");
}

// ----------------- LOOP -----------------
void loop() {
  readBME280();
  readGPS();
  readMPU9250();

  Serial.print("Sending packet #"); Serial.println(counter);
  printPacket(packet);

  // Send binary struct
  LoRa.beginPacket();
  LoRa.write((uint8_t*)&packet, sizeof(packet));
  LoRa.endPacket();

  counter++;
  delay(delayTime);
}
