#include <SPI.h>
#include <LoRa.h>

// --- Pin mapping (your layout)
static const int PIN_SCK  = 18;
static const int PIN_MISO = 19;
static const int PIN_MOSI = 23;
static const int PIN_SS   = 5;   // NSS/CS
static const int PIN_RST  = 14;  // RESET
static const int PIN_DIO0 = 2;   // DIO0

// --- LoRa RF params
long LORA_FREQ_HZ = 433E6;   // MX ISM (change to 433E6 if needed)
int  SPREADING    = 7;       // SF7
long BANDWIDTH    = 125E3;   // 125 kHz
int  CODING_RATE  = 5;       // 4/5
int  TX_POWER_DBM = 17;      // PA_BOOST (be mindful of current draw)

unsigned long counter = 0;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Bring up SPI explicitly with your pins
  SPI.begin(PIN_SCK, PIN_MISO, PIN_MOSI, PIN_SS);

  // Tell LoRa which control pins we’re using
  LoRa.setPins(PIN_SS, PIN_RST, PIN_DIO0);

  Serial.println(F("\n[LoRa TX] Booting..."));
  if (!LoRa.begin(LORA_FREQ_HZ)) {
    Serial.println(F("LoRa init failed. Check wiring/freq."));
    while (true) { delay(1000); }
  }

  // Basic PHY setup
  LoRa.setSpreadingFactor(SPREADING);
  LoRa.setSignalBandwidth(BANDWIDTH);
  LoRa.setCodingRate4(CODING_RATE);
  LoRa.setPreambleLength(8);
  LoRa.setSyncWord(0x34);  // common public network word
  LoRa.enableCrc();        // enable CRC so RX can validate frames
  LoRa.setTxPower(TX_POWER_DBM); // uses PA_BOOST path on SX1278

  Serial.print(F("LoRa ready @ "));
  Serial.print(LORA_FREQ_HZ/1e6);
  Serial.println(F(" MHz"));
}

void loop() {
  // Build a simple message
  String msg = String("Hello from ESP32 #") + counter;

  Serial.print(F("TX -> "));
  Serial.println(msg);

  LoRa.beginPacket();
  LoRa.print(msg);
  LoRa.endPacket();  // blocking until sent

  counter++;
  delay(1000);  // ~1 packet/sec
}
