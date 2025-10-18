#include <SPI.h>
#include <LoRa.h>
#include <math.h>

// ----------------- LoRa (RA-02 @ 433 MHz) -----------------
#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  2

// --- Must match the transmitter ---
#define LORA_FREQ_HZ        433E6
#define LORA_BW_HZ          125E3
#define LORA_SF             7               // same as TX
#define LORA_CR_DENOM       5               // coding rate 4/5
#define LORA_SYNC_WORD      0xF3            // same as TX
#define LORA_PREAMBLE_SYMS  6               // TX uses 6

// -------- PACKET STRUCT (must match TX) ------------
struct SensorPacket {
  float temperature;
  float pressure;
  float altitude;
  float humidity;

  float latitude;
  float longitude;
  float speed;        // km/h
  float gpsAltitude;  // m
  float hdop;
  uint8_t satellites;

  float accelX, accelY, accelZ, accelSqrt; // g
  float gyroX,  gyroY,  gyroZ;             // °/s
  float magX,   magY,   magZ;              // µT
  float magDir;                             // degrees
};

// Pretty printer
void printPacket(const SensorPacket& p) {
  Serial.println("==== PACKET ====");
  Serial.println("--- BME280 ---");
  Serial.print("Temp: "); Serial.println(p.temperature, 2);
  Serial.print("Pres: "); Serial.println(p.pressure, 2);
  Serial.print("Alt:  "); Serial.println(p.altitude, 2);
  Serial.print("Hum:  "); Serial.println(p.humidity, 2);

  Serial.println("--- GPS ---");
  Serial.print("Lat:  "); Serial.println(p.latitude, 6);
  Serial.print("Lon:  "); Serial.println(p.longitude, 6);
  Serial.print("Speed:"); Serial.println(p.speed, 2);
  Serial.print("GPS Alt: "); Serial.println(p.gpsAltitude, 2);
  Serial.print("HDOP: "); Serial.println(p.hdop, 2);
  Serial.print("Sats: "); Serial.println(p.satellites);

  Serial.println("--- IMU (acc g, gyro °/s) ---");
  Serial.print("Accel XYZ: "); Serial.print(p.accelX,3); Serial.print(", ");
  Serial.print(p.accelY,3); Serial.print(", "); Serial.println(p.accelZ,3);
  Serial.print("Accel |v|: "); Serial.println(p.accelSqrt,3);

  Serial.print("Gyro  XYZ: "); Serial.print(p.gyroX,2); Serial.print(", ");
  Serial.print(p.gyroY,2); Serial.print(", "); Serial.println(p.gyroZ,2);

  Serial.println("--- MAG (µT) ---");
  Serial.print("Mag XYZ: "); Serial.print(p.magX,2); Serial.print(", ");
  Serial.print(p.magY,2); Serial.print(", "); Serial.println(p.magZ,2);
  Serial.print("Heading: "); Serial.println(p.magDir, 1);

  Serial.println("================");
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("=== LoRa Receiver (implicit header, preamble=6) ===");

  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  int attempts = 0;
  while (!LoRa.begin(LORA_FREQ_HZ)) {
    Serial.printf("LoRa init attempt %d failed\n", ++attempts);
    delay(300);
  }

  // Modem settings to match TX
  LoRa.setSpreadingFactor(LORA_SF);
  LoRa.setSignalBandwidth(LORA_BW_HZ);
  LoRa.setCodingRate4(LORA_CR_DENOM);
  LoRa.setPreambleLength(LORA_PREAMBLE_SYMS);
  LoRa.setSyncWord(LORA_SYNC_WORD);
  LoRa.enableCrc();                  // TX has CRC enabled
  LoRa.setSPIFrequency(8E6);         // faster SPI host side

  Serial.println("LoRa RX ready (SF7/BW125k/CR4:5, preamble=6, CRC, Sync 0xF3).");
}

void loop() {
  // Implicit header: tell parsePacket the expected size up front
  const size_t EXPECTED = sizeof(SensorPacket);
  int packetSize = LoRa.parsePacket(EXPECTED);   // implicit header mode
  if (packetSize <= 0) return;

  int rssi = LoRa.packetRssi();
  float snr = LoRa.packetSnr();

  SensorPacket pkt;
  int read = LoRa.readBytes((uint8_t*)&pkt, EXPECTED);

  Serial.println();
  Serial.printf("[LoRa] Packet: %d bytes (expected %u), RSSI=%d dBm, SNR=%.1f dB\n",
                read, (unsigned)EXPECTED, rssi, snr);

  if (read != (int)EXPECTED) {
    Serial.println("[WARN] Payload size mismatch with SensorPacket.");
    // Drain any leftover bytes (shouldn't happen in implicit mode)
    while (LoRa.available()) (void)LoRa.read();
    return;
  }

  // Display the packet
  printPacket(pkt);
}
