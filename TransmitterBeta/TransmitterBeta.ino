#include <SPI.h>
#include <LoRa.h>

// LoRa pins
#define ss 5
#define rst 14
#define dio0 2

int counter = 0;
int initAttempts = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("=== LoRa Sender with Debug ===");

  LoRa.setPins(ss, rst, dio0);

  // Try initializing LoRa, show attempt number
  while (!LoRa.begin(433E6)) { // Change to your frequency if needed
    initAttempts++;
    Serial.print("LoRa init attempt ");
    Serial.print(initAttempts);
    Serial.println(" failed.");
    delay(1000); // wait 1 second before next attempt
  }

  LoRa.setSyncWord(0xF3); // Make sure it matches receiver
  Serial.println("LoRa Initialized Successfully!");
}

void loop() {
  // Create message
  String message = "hello " + String(counter);

  // Show debug info before sending
  Serial.print("Sending packet: ");
  Serial.println(message);

  // Send LoRa packet
  LoRa.beginPacket();
  LoRa.print(message);
  LoRa.endPacket();

  // Show confirmation
  Serial.println("Packet sent!");

  counter++;
  delay(10000); // wait 10 seconds before next message
}
