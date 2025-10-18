#include <SPI.h>
#include <LoRa.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++.h>
#include <math.h>

// ----------------- LoRa (RA-02 @ 433 MHz) -----------------
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  2

// ----------------- I2C pins (ESP32) -----------------
#define I2C_SDA 21   // SDA (D21)
#define I2C_SCL 22   // SCL (D22)

// ----------------- BME280 (I2C) -----------------
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme; // I2C

// ----------------- GPS (NEO-6M) -----------------
#define RXD2 16           // ESP32 RX2 <= GPS TX
#define TXD2 17           // ESP32 TX2 => GPS RX
#define GPS_BAUD 9600
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);

// ----------------- IMU (MPU-6050/6500/925x, ICM-206xx) -----------------
#define IMU_ADDR0 0x68
#define IMU_ADDR1 0x69

// WHO_AM_I IDs
enum : uint8_t {
  WHO_MPU6500 = 0x70,
  WHO_MPU6050 = 0x68, // GY-521
  WHO_MPU9250 = 0x71,
  WHO_MPU9255 = 0x73,
  WHO_ICM20600= 0x11,
  WHO_ICM20602= 0x12,
  WHO_ICM20608= 0xAF,
  WHO_ICM20689= 0x98
};

// Common registers (MPU-xxx)
#define REG_WHO_AM_I      0x75
#define REG_PWR_MGMT_1    0x6B
#define REG_SMPLRT_DIV    0x19
#define REG_CONFIG        0x1A
#define REG_GYRO_CONFIG   0x1B
#define REG_ACCEL_CONFIG  0x1C
#define REG_ACCEL_CONFIG2 0x1D   // not present on MPU-6050 (different meaning)
#define REG_ACCEL_XOUT_H  0x3B   // 14 bytes (Ax,Ay,Az,Temp,Gx,Gy,Gz)
#define REG_INT_PIN_CFG   0x37   // BYPASS_EN bit
#define REG_USER_CTRL     0x6A   // I2C_MST_EN bit

// AK8963 magnetometer (inside MPU-9250)
#define AK8963_ADDR   0x0C
#define AK8963_ST1    0x02
#define AK8963_HXL    0x03 // HXL,HXH,HYL,HYH,HZL,HZH
#define AK8963_ST2    0x09
#define AK8963_CNTL1  0x0A
#define AK8963_CNTL2  0x0B
#define AK8963_ASAX   0x10 // ASAX, ASAY, ASAZ

// Scales (±2 g, ±250 °/s)
#define ACC_SENS_2G   16384.0f   // LSB/g
#define GYRO_SENS_250 131.0f     // LSB/(°/s)

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

  float accelX, accelY, accelZ, accelSqrt; // g
  float gyroX,  gyroY,  gyroZ;             // °/s

  float magX,   magY,   magZ;              // µT
  float magDir;                             // degrees [0..360)
};
SensorPacket packet;

// ------------- Scheduler -------------
unsigned long lastSend = 0;
const unsigned long sendPeriod = 2000; // ms
int counter = 0;

// ------------- IMU state -------------
uint8_t imuAddr = IMU_ADDR0;
uint8_t imuWHO  = 0x00;
bool    imuOk   = false;

// AK8963 state (for MPU-9250/9255)
bool    magOk   = false;
float   magAdjX = 1.0f, magAdjY = 1.0f, magAdjZ = 1.0f; // factory ASA scale

// -------- I2C helpers --------
bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return (Wire.endTransmission() == 0);
}
bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false; // repeated start
  if (Wire.requestFrom((int)addr, (int)len) != (int)len) return false;
  for (size_t i = 0; i < len; ++i) buf[i] = Wire.read();
  return true;
}

// ---- AK8963 setup (called when WHO_AM_I = 0x71/0x73) ----
bool ak8963Begin() {
  i2cWrite8(imuAddr, REG_USER_CTRL, 0x00); // disable I2C master
  delay(10);
  i2cWrite8(imuAddr, REG_INT_PIN_CFG, 0x02); // BYPASS_EN
  delay(10);

  i2cWrite8(AK8963_ADDR, AK8963_CNTL1, 0x00); delay(10);
  i2cWrite8(AK8963_ADDR, AK8963_CNTL1, 0x1F); delay(10);

  uint8_t asa[3] = {0};
  if (!i2cRead(AK8963_ADDR, AK8963_ASAX, asa, 3)) return false;

  magAdjX = ((float)asa[0] - 128.0f) / 256.0f + 1.0f;
  magAdjY = ((float)asa[1] - 128.0f) / 256.0f + 1.0f;
  magAdjZ = ((float)asa[2] - 128.0f) / 256.0f + 1.0f;

  i2cWrite8(AK8963_ADDR, AK8963_CNTL1, 0x00); delay(10);
  if (!i2cWrite8(AK8963_ADDR, AK8963_CNTL1, 0x16)) return false; // 16-bit, 100 Hz
  delay(10);

  uint8_t dummy;
  i2cRead(AK8963_ADDR, AK8963_ST2, &dummy, 1);
  return true;
}

// -------- IMU init + detect (adds 9250 mag setup) --------
bool imuBeginGeneric() {
  delay(10);
  for (uint8_t attempt = 0; attempt < 2; ++attempt) {
    imuAddr = (attempt == 0) ? IMU_ADDR0 : IMU_ADDR1;

    if (!i2cRead(imuAddr, REG_WHO_AM_I, &imuWHO, 1)) continue;

    bool known =
      (imuWHO == WHO_MPU6500) || (imuWHO == WHO_MPU6050) ||
      (imuWHO == WHO_MPU9250) || (imuWHO == WHO_MPU9255) ||
      (imuWHO == WHO_ICM20600)|| (imuWHO == WHO_ICM20602)||
      (imuWHO == WHO_ICM20608)|| (imuWHO == WHO_ICM20689);

    Serial.printf("[IMU] WHO_AM_I=0x%02X @ 0x%02X %s\n",
                  imuWHO, imuAddr, known ? "(known)" : "(unknown)");
    if (!known) return false;

    // Reset, then select PLL clock
    i2cWrite8(imuAddr, REG_PWR_MGMT_1, 0x80); // device reset
    delay(100);
    i2cWrite8(imuAddr, REG_PWR_MGMT_1, 0x01); // clock = PLL
    delay(10);

    // Gyro DLPF (~42–44 Hz) for all
    i2cWrite8(imuAddr, REG_CONFIG, 0x03);

    // ACCEL_CONFIG2 is NOT for MPU-6050; skip there
    bool is6050 = (imuWHO == WHO_MPU6050);
    if (!is6050) {
      i2cWrite8(imuAddr, REG_ACCEL_CONFIG2, 0x03);
    }

    // Sample rate: 1 kHz / (1 + 9) = 100 Hz
    i2cWrite8(imuAddr, REG_SMPLRT_DIV, 0x09);

    // FS ranges: ±250 dps, ±2 g
    i2cWrite8(imuAddr, REG_GYRO_CONFIG,  0x00);
    i2cWrite8(imuAddr, REG_ACCEL_CONFIG, 0x00);

    // If this is an MPU-9250/9255, bring up the AK8963 mag
    if (imuWHO == WHO_MPU9250 || imuWHO == WHO_MPU9255) {
      magOk = ak8963Begin();
      Serial.println(magOk ? "AK8963 mag ✔ (16-bit, 100 Hz)" : "AK8963 mag ✖");
    } else {
      magOk = false;
    }

    return true;
  }
  return false;
}

bool imuReadAG(float &ax_g, float &ay_g, float &az_g, float &gx_dps, float &gy_dps, float &gz_dps) {
  uint8_t raw[14];
  if (!i2cRead(imuAddr, REG_ACCEL_XOUT_H, raw, sizeof(raw))) return false;

  auto toI16 = [](uint8_t hi, uint8_t lo)->int16_t { return (int16_t)((hi << 8) | lo); };
  int16_t ax = toI16(raw[0],  raw[1]);
  int16_t ay = toI16(raw[2],  raw[3]);
  int16_t az = toI16(raw[4],  raw[5]);
  int16_t gx = toI16(raw[8],  raw[9]);
  int16_t gy = toI16(raw[10], raw[11]);
  int16_t gz = toI16(raw[12], raw[13]);

  ax_g = (float)ax / ACC_SENS_2G;
  ay_g = (float)ay / ACC_SENS_2G;
  az_g = (float)az / ACC_SENS_2G;
  gx_dps = (float)gx / GYRO_SENS_250;
  gy_dps = (float)gy / GYRO_SENS_250;
  gz_dps = (float)gz / GYRO_SENS_250;
  return true;
}

bool ak8963ReadMag(float &mx_uT, float &my_uT, float &mz_uT) {
  if (!magOk) return false;

  uint8_t st1 = 0;
  if (!i2cRead(AK8963_ADDR, AK8963_ST1, &st1, 1)) return false;
  if ((st1 & 0x01) == 0) return false; // data not ready

  uint8_t raw[7]; // HXL,HXH,HYL,HYH,HZL,HZH,ST2
  if (!i2cRead(AK8963_ADDR, AK8963_HXL, raw, sizeof(raw))) return false;

  // Check overflow
  uint8_t st2 = raw[6];
  if (st2 & 0x08) return false;

  auto toI16_LE = [](uint8_t lo, uint8_t hi)->int16_t { return (int16_t)((hi << 8) | lo); };
  int16_t hx = toI16_LE(raw[0], raw[1]);
  int16_t hy = toI16_LE(raw[2], raw[3]);
  int16_t hz = toI16_LE(raw[4], raw[5]);

  // 16-bit mode scale: 0.15 µT/LSB, apply factory ASA adjustment
  const float mRes = 0.15f;
  mx_uT = (float)hx * magAdjX * mRes;
  my_uT = (float)hy * magAdjY * mRes;
  mz_uT = (float)hz * magAdjZ * mRes;

  return true;
}

// -------- App sensors --------
void readBME280() {
  packet.temperature = bme.readTemperature();
  packet.pressure    = bme.readPressure() / 100.0F;
  packet.altitude    = bme.readAltitude(SEALEVELPRESSURE_HPA);
  packet.humidity    = bme.readHumidity();
}

void readGPS() {
  if (gps.location.isValid()) {
    packet.latitude  = gps.location.lat();
    packet.longitude = gps.location.lng();
  } else {
    packet.latitude  = 0;
    packet.longitude = 0;
  }
  packet.speed       = gps.speed.isValid()     ? gps.speed.kmph()      : 0;
  packet.gpsAltitude = gps.altitude.isValid()  ? gps.altitude.meters() : 0;
  packet.hdop        = gps.hdop.isValid()      ? gps.hdop.hdop()       : 0;
  packet.satellites  = gps.satellites.isValid()? (uint8_t)gps.satellites.value() : 0;

  static bool warnedNoFix = false;
  if (!gps.location.isValid()) {
    if (!warnedNoFix) { Serial.println("[GPS] No fix yet..."); warnedNoFix = true; }
  } else {
    warnedNoFix = false;
  }
}

void readIMU() {
  if (!imuOk) {
    packet.accelX = packet.accelY = packet.accelZ = packet.accelSqrt = 0;
    packet.gyroX  = packet.gyroY  = packet.gyroZ  = 0;
    packet.magX = packet.magY = packet.magZ = 0;
    packet.magDir = 0;
    return;
  }
  float ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps;
  if (imuReadAG(ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps)) {
    packet.accelX = ax_g;
    packet.accelY = ay_g;
    packet.accelZ = az_g;
    packet.accelSqrt = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
    packet.gyroX = gx_dps; packet.gyroY = gy_dps; packet.gyroZ = gz_dps;
  } else {
    Serial.println("[IMU] Accel/Gyro read failed");
  }

  // Magnetometer (if available)
  float mx, my, mz;
  if (ak8963ReadMag(mx, my, mz)) {
    packet.magX = mx; packet.magY = my; packet.magZ = mz;
    float heading = atan2f(packet.magY, packet.magX) * 180.0f / (float)M_PI;
    if (heading < 0) heading += 360.0f;
    packet.magDir = heading;
  } else {
    packet.magX = packet.magY = packet.magZ = 0;
    packet.magDir = 0;
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }
  Serial.println("\nLoRa + BME280 + GPS + IMU (MPU-9250 ready) Sender");

  // I2C @ 400 kHz on your pins
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // BME280 0x76 -> 0x77 fallback
  bool bmeOK = bme.begin(0x76);
  if (!bmeOK) bmeOK = bme.begin(0x77);
  Serial.println(bmeOK ? "BME280 ✔" : "[BME280] Not found (0x76/0x77)");

  // GPS UART2 (GPS TX->16, RX<-17)
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  Serial.println("GPS UART2 ✔");

  // IMU (auto-detect; if 9250/55, sets up AK8963 mag)
  imuOk = imuBeginGeneric();
  if (imuOk) {
    Serial.printf("IMU ✔ (WHO_AM_I=0x%02X @ 0x%02X)\n", imuWHO, imuAddr);
  } else {
    Serial.println("IMU ✖ (Check VCC/GND, SDA=21, SCL=22, AD0 strap).");
  }

  // LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  while (!LoRa.begin(433E6)) {
    Serial.println("LoRa init...");
    delay(300);
  }
  // === Range-neutral speed-ups ===
  LoRa.setSpreadingFactor(7);             // keep SF7
  LoRa.setSignalBandwidth(125E3);         // keep 125 kHz
  LoRa.setCodingRate4(5);                 // keep 4/5
  LoRa.setPreambleLength(6);              // ↓ from default 8 to 6 (slightly shorter)
  LoRa.setSyncWord(0xF3);
  LoRa.enableCrc();                       // keep CRC
  LoRa.setTxPower(17);
  LoRa.setSPIFrequency(8E6);              // faster SPI to radio (CPU-side)

  Serial.println("LoRa ✔ (implicit hdr, preamble=6, SF7/BW125k/CR4:5, CRC, Sync 0xF3)");

  static_assert(sizeof(SensorPacket) <= 255, "SensorPacket too large for single LoRa.write()");
}

void loop() {
  // Continuous GPS feed (non-blocking)
  while (gpsSerial.available()) gps.encode(gpsSerial.read());

  // Periodic read + send
  if (millis() - lastSend >= sendPeriod) {
    readBME280();
    readGPS();
    readIMU();

    // Minimal prints to avoid stalling
    // Serial.print("TX #"); Serial.println(counter);

    // Implicit header -> smaller overhead (range-neutral)
    LoRa.beginPacket(true); // true = implicit header, RX must know payload size
    LoRa.write(reinterpret_cast<uint8_t*>(&packet), sizeof(packet));
    LoRa.endPacket();       // blocking send; simple & safe

    counter++;
    lastSend = millis();
  }
}

